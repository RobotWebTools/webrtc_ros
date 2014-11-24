#include <ros/ros.h>
#include <ros/package.h>
#include <webrtc_ros/webrtc_ros_server.h>
#include <cpp_web_server/http_reply.hpp>
#include <cpp_web_server/websocket_request_handler.hpp>
#include "webrtc/base/ssladapter.h"

namespace webrtc_ros {

static void ros_connection_logger(cpp_web_server::HttpServerRequestHandler forward,
				  const cpp_web_server::HttpRequest &request,
				  cpp_web_server::HttpConnectionPtr connection,
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

cpp_web_server::WebsocketConnection::MessageHandler WebrtcRosServer::handle_webrtc_websocket(const cpp_web_server::HttpRequest& request,
											     cpp_web_server::WebsocketConnectionPtr websocket) {
  ROS_INFO_STREAM("Handling new WebRTC websocket");
  boost::shared_ptr<WebrtcClient> client(new WebrtcClient(nh_, websocket));
  clients_.push_back(client);
  return client->createMessageHandler();
}

WebrtcRosServer::WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh),
    handler_group_(cpp_web_server::HttpReply::stock_reply(cpp_web_server::HttpReply::not_found)){
  rtc::InitializeSSL();

  int port;
  pnh_.param("port", port, 8080);

  handler_group_.addHandlerForPath("/", cpp_web_server::HttpReply::from_file(cpp_web_server::HttpReply::ok, "text/html",
									     ros::package::getPath("webrtc_ros")+"/web/test.html"));
  handler_group_.addHandlerForPath("/webrtc", cpp_web_server::WebsocketHttpRequestHandler(boost::bind(&WebrtcRosServer::handle_webrtc_websocket, this, _1, _2)));
  server_.reset(new cpp_web_server::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port),
					       boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4), 1));

}
WebrtcRosServer::~WebrtcRosServer(){
  rtc::CleanupSSL();
}

void WebrtcRosServer::spin() {
  server_->run();
  ROS_INFO("Waiting For connections");
  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();
  server_->stop();
}

}
