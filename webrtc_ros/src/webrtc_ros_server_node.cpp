#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/webrtc_ros_server.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("webrtc_ros_server");

  webrtc_ros::WebrtcRosServer server(node);
  server.run();
    
  rclcpp::spin(node);

  server.stop();

  rclcpp::shutdown();
}
