#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/webrtc_ros_server.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/shared_ptr.hpp>

namespace webrtc_ros
{
  class WebrtcRosServerNodelet : public nodelet::Nodelet
{
public:
  ~WebrtcRosServerNodelet()
  {
    if(server_)
    {
      server_->stop();
    }
  }

  void onInit()
  {
    server_.reset(new WebrtcRosServer(getNodeHandle(), getPrivateNodeHandle()));
    server_->run();
  }
private:
  boost::shared_ptr<WebrtcRosServer> server_;
};


}

PLUGINLIB_EXPORT_CLASS(webrtc_ros::WebrtcRosServerNodelet, nodelet::Nodelet);

