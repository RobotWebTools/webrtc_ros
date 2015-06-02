#ifndef WEBRTC_ROS_ROS_VIDEO_CAPTURER_H_
#define WEBRTC_ROS_ROS_VIDEO_CAPTURER_H_

#include "talk/media/base/videocapturer.h"
#include "talk/media/base/videocapturerfactory.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <boost/enable_shared_from_this.hpp>

namespace webrtc_ros
{


class RosVideoCapturer;
class RosVideoCapturerImpl;

class ImageMessageHandler : public rtc::MessageHandler {
public:
  ImageMessageHandler(RosVideoCapturer *capturer);
  virtual void OnMessage(rtc::Message* msg);
private:
  RosVideoCapturer *capturer_;
};

class RosVideoCapturer :
  public cricket::VideoCapturer
{
public:
  RosVideoCapturer(image_transport::ImageTransport it, const std::string& topic);
  virtual ~RosVideoCapturer();

  void imageCallback(cricket::CapturedFrame *frame);

  virtual cricket::CaptureState Start(const cricket::VideoFormat& capture_format) override;
  virtual void Stop() override;
  virtual bool IsRunning() override;
  virtual bool GetPreferredFourccs(std::vector<uint32>* fourccs) override;
  virtual bool GetBestCaptureFormat(const cricket::VideoFormat& desired,
                                    cricket::VideoFormat* best_format) override;
  virtual bool IsScreencast() const override;

private:
  DISALLOW_COPY_AND_ASSIGN(RosVideoCapturer);

  void SignalFrameCapturedOnStartThread(cricket::CapturedFrame *frame);

  rtc::Thread* volatile start_thread_;
  ImageMessageHandler handler_;
  boost::shared_ptr<RosVideoCapturerImpl> impl_;

  friend ImageMessageHandler;
};


// The Impl class represents the actual backend that is receiving ROS events
// It is needed because the VideoCapturer can be destroyed while there is still
// an image callback queued/running, leading to undefined behavior
class RosVideoCapturerImpl : public boost::enable_shared_from_this<RosVideoCapturerImpl>
{
public:
  RosVideoCapturerImpl(image_transport::ImageTransport it, const std::string& topic);
  virtual ~RosVideoCapturerImpl();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void Start(RosVideoCapturer *capturer);
  void Stop();

private:
  DISALLOW_COPY_AND_ASSIGN(RosVideoCapturerImpl);

  image_transport::ImageTransport it_;
  const std::string topic_;
  image_transport::Subscriber sub_;
  std::mutex state_mutex_;
  RosVideoCapturer *capturer_;
};



}


#endif
