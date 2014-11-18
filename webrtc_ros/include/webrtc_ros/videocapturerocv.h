#ifndef SCY_VideoCaturerOCV_H
#define SCY_VideoCaturerOCV_H

#include "talk/media/base/videocapturer.h"
#include "talk/media/base/videocapturerfactory.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace scy { 


  class VideoCapturerOCV : 
    public cricket::VideoCapturer
    {
    public:
      VideoCapturerOCV(int deviceId);
      virtual ~VideoCapturerOCV();

      // Overrides from VideoCapture.
      void onFrameCaptured(const sensor_msgs::ImageConstPtr& msg);

      // cricket::VideoCapturer implementation.
      virtual cricket::CaptureState Start(
					  const cricket::VideoFormat& capture_format) OVERRIDE;
      virtual void Stop() OVERRIDE;
      virtual bool IsRunning() OVERRIDE;
      virtual bool GetPreferredFourccs(std::vector<uint32>* fourccs) OVERRIDE;
      virtual bool GetBestCaptureFormat(const cricket::VideoFormat& desired,
					cricket::VideoFormat* best_format) OVERRIDE;
      virtual bool IsScreencast() const OVERRIDE;

    private:
      DISALLOW_COPY_AND_ASSIGN(VideoCapturerOCV);
      image_transport::Subscriber sub;
    };


  class VideoCapturerFactoryOCV : public cricket::VideoDeviceCapturerFactory 
  {
  public:
    VideoCapturerFactoryOCV() {}
    virtual ~VideoCapturerFactoryOCV() {}

    virtual cricket::VideoCapturer* Create(const cricket::Device& device) {
      // XXX: WebRTC uses device name to instantiate the capture, which is always 0.
      //return new VideoCapturerOCV(webrtc::strtoi<int>(device.id));
      return new VideoCapturerOCV(0);
    }
  };


} // namespace scy


#endif
