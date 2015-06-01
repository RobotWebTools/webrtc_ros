#ifndef WEBRTC_ROS_WEBRTC_ROS_SERVER_H_
#define WEBRTC_ROS_WEBRTC_ROS_SERVER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <webrtc_ros/webrtc_client.h>
#include <webrtc_ros/ros_log_context.h>
#include <webrtc_ros/webrtc_web_server.h>

namespace webrtc_ros
{

class WebrtcRosServer
{
public:
  WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~WebrtcRosServer();
  void run();
  void stop();

  MessageHandler* handle_new_signaling_channel(SignalingChannel *channel);
private:
  std::vector<WebrtcClientWeakPtr> clients_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::shared_ptr<webrtc_ros::WebrtcWebServer> server_;
  RosLogContext log_context_;
};

}

#endif
