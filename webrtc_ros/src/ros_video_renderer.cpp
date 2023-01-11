#include <webrtc_ros/ros_video_renderer.h>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <webrtc/3rdparty/libyuv/convert_from.h>

namespace webrtc_ros
{

RosVideoRenderer::RosVideoRenderer(std::shared_ptr<image_transport::ImageTransport> it, const std::string& topic)
  : it_(it)
  , topic_(topic)
{
  pub_ = it_->advertise(topic_, 1);
}

void RosVideoRenderer::OnFrame(const webrtc::VideoFrame& frame)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock().now();

  const rtc::scoped_refptr<webrtc::I420BufferInterface>& buffer = frame.video_frame_buffer()->ToI420();

  cv::Mat bgra(buffer->height(), buffer->width(), CV_8UC4);
  // The ARGB function in libyuv appears to output BGRA...
  libyuv::I420ToARGB(buffer->DataY(), buffer->StrideY(),
                     buffer->DataU(), buffer->StrideU(),
                     buffer->DataV(), buffer->StrideV(),
                     bgra.data, bgra.step, buffer->width(), buffer->height());

  cv_bridge::CvImage image(header, "bgra8", bgra);
  pub_.publish(image.toImageMsg());
}

}
