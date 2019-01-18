#ifndef WEBRTC_ROS_WEBRTC_ROS_SERVER_H_
#define WEBRTC_ROS_WEBRTC_ROS_SERVER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <webrtc_ros/webrtc_client.h>
#include <webrtc_ros/ros_log_context.h>
#include <webrtc_ros/webrtc_web_server.h>
#include <condition_variable>

namespace webrtc_ros
{

MessageHandler* WebrtcRosServer_handle_new_signaling_channel(void* _this, SignalingChannel *channel);

class WebrtcRosServer
{
public:
  WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~WebrtcRosServer();
  void run();
  void stop();

  MessageHandler* handle_new_signaling_channel(SignalingChannel *channel);
  void cleanupWebrtcClient(WebrtcClient *client);

  rtc::Thread signaling_thread_;
private:
  RosLogContextRef log_context_;

  std::condition_variable shutdown_cv_;
  std::mutex clients_mutex_;
  std::map<WebrtcClient*, WebrtcClientWeakPtr> clients_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string image_transport_;
  ImageTransportFactory itf_;

  boost::shared_ptr<webrtc_ros::WebrtcWebServer> server_;
};

}

#endif
