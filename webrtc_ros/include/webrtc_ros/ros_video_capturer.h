#ifndef WEBRTC_ROS_ROS_VIDEO_CAPTURER_H_
#define WEBRTC_ROS_ROS_VIDEO_CAPTURER_H_

#include <webrtc/media/base/videocapturer.h>
#include <webrtc/media/base/videocapturerfactory.h>
#include <webrtc/base/event.h>
#include <webrtc/base/thread.h>
#include <webrtc_ros/image_transport_factory.h>
#include <mutex>
#include <boost/enable_shared_from_this.hpp>


namespace webrtc_ros
{

class RosVideoCapturerImpl;

class RosVideoCapturer :
  public cricket::VideoCapturer
{
public:
  RosVideoCapturer(const ImageTransportFactory& it, const std::string& topic, const std::string& transport);
  virtual ~RosVideoCapturer();

  void imageCallback(const std::shared_ptr<webrtc::VideoFrame>& frame);

  virtual cricket::CaptureState Start(const cricket::VideoFormat& capture_format) override;
  virtual void Stop() override;
  virtual bool IsRunning() override;
  virtual bool GetPreferredFourccs(std::vector<uint32_t>* fourccs) override;
  virtual bool GetBestCaptureFormat(const cricket::VideoFormat& desired,
                                    cricket::VideoFormat* best_format) override;
  virtual bool IsScreencast() const override;

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

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

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
