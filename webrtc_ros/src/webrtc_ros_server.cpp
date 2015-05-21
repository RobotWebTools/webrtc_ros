#include <ros/ros.h>
#include <webrtc_ros/webrtc_ros_server.h>
#include "webrtc/base/ssladapter.h"
#include <boost/foreach.hpp>

namespace webrtc_ros
{

MessageHandler* WebrtcRosServer_handle_new_signaling_channel(void* _this, SignalingChannel *channel) {
  return ((WebrtcRosServer*)_this)->handle_new_signaling_channel(channel);
}

WebrtcRosServer::WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  rtc::InitializeSSL();

  int port;
  pnh_.param("port", port, 8080);

  server_.reset(WebrtcWebServer::create(port, &WebrtcRosServer_handle_new_signaling_channel, this));
}

MessageHandler* WebrtcRosServer::handle_new_signaling_channel(SignalingChannel *channel)
{
  boost::shared_ptr<WebrtcClient> client(new WebrtcClient(nh_, channel));
  // hold a shared ptr until the object is initialized
  client->init();
  clients_.push_back(client);
  return client->createMessageHandler();
}

WebrtcRosServer::~WebrtcRosServer()
{
  BOOST_FOREACH(WebrtcClientWeakPtr client_weak, clients_)
  {
    boost::shared_ptr<WebrtcClient> client = client_weak.lock();
    if (client)
      client->invalidate();
  }
  // TODO: should call stop here, but right now it will fail if stop has already been called
  // This is a bug in async_web_server_cpp
  //stop();
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
