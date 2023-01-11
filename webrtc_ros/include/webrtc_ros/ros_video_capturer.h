#ifndef WEBRTC_ROS_ROS_VIDEO_CAPTURER_H_
#define WEBRTC_ROS_ROS_VIDEO_CAPTURER_H_

#include <webrtc/modules/video_capture/video_capture.h>
#include <webrtc/modules/video_capture/video_capture_factory.h>
#include <webrtc/media/base/adapted_video_track_source.h>
#include <webrtc/api/video/video_source_interface.h>
#include <webrtc/api/video/i420_buffer.h>
#include <webrtc/rtc_base/event.h>
#include <webrtc/rtc_base/thread.h>
#include <webrtc_ros/image_transport_factory.h>
#include <mutex>
#include <boost/enable_shared_from_this.hpp>


namespace webrtc_ros
{

class RosVideoCapturerImpl;

class RosVideoCapturer :
  public rtc::AdaptedVideoTrackSource
{
public:
  RosVideoCapturer(const ImageTransportFactory& it, const std::string& topic, const std::string& transport);
  ~RosVideoCapturer() override;

  void imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg);
  void Start();

  void Stop();

  bool is_screencast() const override;
  absl::optional<bool> needs_denoising() const override;
  void SetState(webrtc::MediaSourceInterface::SourceState state);
	webrtc::MediaSourceInterface::SourceState state() const override;
	bool remote() const override;

private:
  RTC_DISALLOW_COPY_AND_ASSIGN(RosVideoCapturer);
  boost::shared_ptr<RosVideoCapturerImpl> impl_;
};


// The Impl class represents the actual backend that is receiving ROS events
// It is needed because the VideoCapturer can be destroyed while there is still
// an image callback queued/running, leading to undefined behavior
class RosVideoCapturerImpl : public boost::enable_shared_from_this<RosVideoCapturerImpl>
{
public:
  RosVideoCapturerImpl(const ImageTransportFactory& it, const std::string& topic, const std::string& transport);

  void imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg);

  void Start(RosVideoCapturer *capturer);
  void Stop();

private:
  RTC_DISALLOW_COPY_AND_ASSIGN(RosVideoCapturerImpl);

  ImageTransportFactory it_;
  const std::string topic_, transport_;
  ImageTransportFactory::Subscriber sub_;
  std::mutex state_mutex_;
  RosVideoCapturer *capturer_;
};



}


#endif
