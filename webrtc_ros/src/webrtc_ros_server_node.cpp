#include <ros/ros.h>
#include <webrtc_ros/webrtc_ros_server.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "webrtc_ros_server");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  webrtc_ros::WebrtcRosServer server(nh, pnh);
  server.run();
  ros::spin();
  server.stop();
}
