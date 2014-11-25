#include <ros/ros.h>
#include <ros/package.h>
#include <webrtc_ros/webrtc_ros_server.h>
#include <async_web_server_cpp/http_reply.hpp>
#include <async_web_server_cpp/websocket_request_handler.hpp>
#include "webrtc/base/ssladapter.h"
#include <boost/foreach.hpp>

namespace webrtc_ros {

static void ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
				  const async_web_server_cpp::HttpRequest &request,
				  async_web_server_cpp::HttpConnectionPtr connection,
				  const char* begin, const char* end)
{
  ROS_INFO_STREAM("Handling Request: " << request.uri);
  try
  {
    forward(request, connection, begin, end);
  } catch (std::exception &e)
  {
    ROS_WARN_STREAM("Error Handling Request: " << e.what());
  }
}

async_web_server_cpp::WebsocketConnection::MessageHandler WebrtcRosServer::handle_webrtc_websocket(const async_web_server_cpp::HttpRequest& request,
											     async_web_server_cpp::WebsocketConnectionPtr websocket) {
  ROS_INFO_STREAM("Handling new WebRTC websocket");
  boost::shared_ptr<WebrtcClient> client(new WebrtcClient(nh_, websocket));
  // hold a shared ptr until the object is initialized
  client->init();
  clients_.push_back(client);
  return client->createMessageHandler();
}

WebrtcRosServer::WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh),
    handler_group_(async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)){
  rtc::InitializeSSL();

  int port;
  pnh_.param("port", port, 8080);

  handler_group_.addHandlerForPath("/", async_web_server_cpp::HttpReply::from_file(async_web_server_cpp::HttpReply::ok, "text/html",
									     ros::package::getPath("webrtc_ros")+"/web/test.html"));
  handler_group_.addHandlerForPath("/webrtc", async_web_server_cpp::WebsocketHttpRequestHandler(boost::bind(&WebrtcRosServer::handle_webrtc_websocket, this, _1, _2)));
  server_.reset(new async_web_server_cpp::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port),
					       boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4), 1));

}
WebrtcRosServer::~WebrtcRosServer(){
  BOOST_FOREACH(WebrtcClientWeakPtr client_weak, clients_) {
    boost::shared_ptr<WebrtcClient> client = client_weak.lock();
    if(client)
      client->invalidate();
  }
  rtc::CleanupSSL();
}

void WebrtcRosServer::spin() {
  server_->run();
  ROS_INFO("Waiting For connections");
  ros::spin();
  server_->stop();
}

}
