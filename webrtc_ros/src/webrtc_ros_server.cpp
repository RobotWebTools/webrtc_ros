#include <ros/ros.h>
#include <webrtc_ros/webrtc_ros_server.h>
#include <cpp_web_server/http_reply.hpp>
#include <cpp_web_server/websocket_request_handler.hpp>

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

static cpp_web_server::WebsocketConnectionPtr websocket_;
static bool x = true;
static void handle_message(const cpp_web_server::WebsocketMessage& message){
  std::cout << "Message: " << message.type << ": " << message.content.size() << std::endl;
  if (x){
    x = false;
  cpp_web_server::WebsocketMessage m;
  m.type = cpp_web_server::WebsocketMessage::type_text;
  m.content = "hello this is a test";
  websocket_->sendMessage(m);
  m.type = cpp_web_server::WebsocketMessage::type_ping;
  m.content = "pingtest";
  websocket_->sendMessage(m);
  }
}

static cpp_web_server::WebsocketConnection::MessageHandler handle_websocket(const cpp_web_server::HttpRequest& request,
							    cpp_web_server::WebsocketConnectionPtr websocket) {
  for(std::vector<cpp_web_server::HttpHeader>::const_iterator itr = request.headers.begin(); itr != request.headers.end(); ++itr) {
    ROS_INFO_STREAM("HEADER: " << itr->name << ": " << itr->value);
  }
  websocket_ = websocket;
  return handle_message;
}

WebrtcRosServer::WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh),
    handler_group_(cpp_web_server::HttpReply::stock_reply(cpp_web_server::HttpReply::not_found)){

  int port;
  pnh_.param("port", port, 8080);


  handler_group_.addHandlerForPath("/ws", cpp_web_server::WebsocketHttpRequestHandler(handle_websocket));
  server_.reset(new cpp_web_server::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port),
					       boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4), 1));

}
void WebrtcRosServer::spin() {
  server_->run();
  ROS_INFO("Waiting For connections");
  ros::MultiThreadedSpinner spinner(1);
  spinner.spin();
  server_->stop();
}

}
