#include "webrtc_ros/ros_video_renderer.h"
#include <talk/media/base/videoframe.h>
#include <libyuv/convert_from.h>

namespace webrtc_ros
{


RosVideoRenderer::RosVideoRenderer(image_transport::ImageTransport it, const std::string& topic)
  : it_(it), topic_(topic)
{
  pub_ = it_.advertise(topic_, 1);
}


RosVideoRenderer::~RosVideoRenderer()
{
}


void RosVideoRenderer::SetSize(int width, int height)
{
  ROS_DEBUG("Render SetSize %d,%d", width, height);
}

void RosVideoRenderer::RenderFrame(const cricket::VideoFrame* frame)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();

  cv::Mat bgra(frame->GetHeight(), frame->GetWidth(), CV_8UC4);
  // The ARGB function in libyuv appears to output BGRA...
  libyuv::I420ToARGB(frame->GetYPlane(), frame->GetYPitch(),
                     frame->GetUPlane(), frame->GetUPitch(),
                     frame->GetVPlane(), frame->GetVPitch(),
                     bgra.data, bgra.step, frame->GetWidth(), frame->GetHeight());


  cv_bridge::CvImage image(header, "bgra8", bgra);
  pub_.publish(image.toImageMsg());
}

}
