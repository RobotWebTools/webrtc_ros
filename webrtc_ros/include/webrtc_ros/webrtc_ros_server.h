#ifndef WEBRTC_ROS_WEBRTC_ROS_SERVER_H_
#define WEBRTC_ROS_WEBRTC_ROS_SERVER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <async_web_server_cpp/http_server.hpp>
#include <async_web_server_cpp/websocket_connection.hpp>
#include <webrtc_ros/webrtc_client.h>

namespace webrtc_ros
{

class WebrtcRosServer
{
public:
  WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~WebrtcRosServer();
  async_web_server_cpp::WebsocketConnection::MessageHandler handle_webrtc_websocket(const async_web_server_cpp::HttpRequest& request,
      async_web_server_cpp::WebsocketConnectionPtr websocket);
  void spin();
private:
  void handle_list_streams(const async_web_server_cpp::HttpRequest &request,
			   async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end);

  std::vector<WebrtcClientWeakPtr> clients_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::shared_ptr<async_web_server_cpp::HttpServer> server_;
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_;
};

}

#endif
