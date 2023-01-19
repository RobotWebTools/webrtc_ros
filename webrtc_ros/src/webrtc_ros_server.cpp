#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/webrtc_ros_server.h>
#include "webrtc/rtc_base/ssl_adapter.h"

#include "webrtc/rtc_base/bind.h"

namespace webrtc_ros
{

MessageHandler* WebrtcRosServer_handle_new_signaling_channel(void* _this, SignalingChannel *channel)
{
    return ((WebrtcRosServer*) _this)->signaling_thread_->Invoke<MessageHandler*>(RTC_FROM_HERE, rtc::Bind(&WebrtcRosServer::handle_new_signaling_channel,
            (WebrtcRosServer*)_this, channel));
}

WebrtcRosServer::WebrtcRosServer(rclcpp::Node::SharedPtr nh)
  : nh_(nh), itf_(nh, std::make_shared<image_transport::ImageTransport>(nh))
{
  rtc::InitializeSSL();

  int port;
  nh_->get_parameter_or<int>("port", port, 8080);
  nh_->get_parameter_or<std::string>("image_transport", image_transport_, std::string("raw"));

  signaling_thread_ = rtc::Thread::CreateWithSocketServer();
  signaling_thread_->Start();
  server_.reset(WebrtcWebServer::create(nh_, port, &WebrtcRosServer_handle_new_signaling_channel, this));
}

void WebrtcRosServer::cleanupWebrtcClient(WebrtcClient *client) {
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    clients_.erase(client);
    delete client; // delete while holding the lock so that we do not exit before were done
  }
  shutdown_cv_.notify_all();
}

MessageHandler* WebrtcRosServer::handle_new_signaling_channel(SignalingChannel *channel)
{
  auto client = std::make_shared<WebrtcClient>(nh_, itf_, image_transport_, channel);

	// TODO: Handle cleanup std::bind(&WebrtcRosServer::cleanupWebrtcClient, this, std::placeholders::_1));
  // hold a shared ptr until the object is initialized (holds a ptr to itself)
  client->init(client);
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    clients_[client.get()] = WebrtcClientWeakPtr(client);
  }
  return client->createMessageHandler();
}

WebrtcRosServer::~WebrtcRosServer()
{
  stop();

  // Send all clients messages to shutdown, cannot call dispose of share ptr while holding clients_mutex_
  // It will deadlock if it is the last shared_ptr because it will try to remove it from the client list
  std::vector<WebrtcClientWeakPtr> to_invalidate;
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    for(auto& client_entry : clients_) {
      to_invalidate.push_back(client_entry.second);
    }
  }
  for(WebrtcClientWeakPtr& client_weak : to_invalidate) {
    std::shared_ptr<WebrtcClient> client = client_weak.lock();
    if (client)
      client->invalidate();
  }

  // Wait for all our clients to shown
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    shutdown_cv_.wait(lock, [this]{ return this->clients_.size() == 0; });
  }

  rtc::CleanupSSL();
}

void WebrtcRosServer::run()
{
  server_->run();
}

void WebrtcRosServer::stop()
{
  server_->stop();
}

}
