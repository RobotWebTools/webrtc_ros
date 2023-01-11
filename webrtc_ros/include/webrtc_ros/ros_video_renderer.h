#ifndef WEBRTC_ROS_ROS_VIDEO_RENDERER_H_
#define WEBRTC_ROS_ROS_VIDEO_RENDERER_H_

#include <webrtc/api/media_stream_interface.h>
#include <image_transport/image_transport.hpp>


namespace webrtc_ros
{

class RosVideoRenderer :
  public rtc::VideoSinkInterface<webrtc::VideoFrame>
{
public:
  RosVideoRenderer(std::shared_ptr<image_transport::ImageTransport> it, const std::string& topic);
  virtual void OnFrame(const webrtc::VideoFrame& frame) override;

private:
  RTC_DISALLOW_COPY_AND_ASSIGN(RosVideoRenderer);
  std::shared_ptr<image_transport::ImageTransport> it_;
  const std::string topic_;
  image_transport::Publisher pub_;
};

}

#endif
