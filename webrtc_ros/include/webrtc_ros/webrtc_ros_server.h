#ifndef WEBRTC_ROS_WEBRTC_ROS_SERVER_H_
#define WEBRTC_ROS_WEBRTC_ROS_SERVER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <cpp_web_server/http_server.hpp>
#include <cpp_web_server/websocket_connection.hpp>
#include <webrtc_ros/webrtc_client.h>

namespace webrtc_ros {

class WebrtcRosServer {
 public:
  WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~WebrtcRosServer();
  cpp_web_server::WebsocketConnection::MessageHandler handle_webrtc_websocket(const cpp_web_server::HttpRequest& request,
									      cpp_web_server::WebsocketConnectionPtr websocket);
  void spin();
 private:
  std::vector<WebrtcClientWeakPtr> clients_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::shared_ptr<cpp_web_server::HttpServer> server_;
  cpp_web_server::HttpRequestHandlerGroup handler_group_;
};

}

#endif
