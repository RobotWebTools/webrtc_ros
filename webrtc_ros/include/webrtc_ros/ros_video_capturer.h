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

namespace webrtc_ros {


  class RosVideoCapturer :
    public cricket::VideoCapturer
    {
    public:
      RosVideoCapturer();
      virtual ~RosVideoCapturer();

      void imageCallback(const sensor_msgs::ImageConstPtr& msg);

      virtual cricket::CaptureState Start(const cricket::VideoFormat& capture_format) OVERRIDE;
      virtual void Stop() OVERRIDE;
      virtual bool IsRunning() OVERRIDE;
      virtual bool GetPreferredFourccs(std::vector<uint32>* fourccs) OVERRIDE;
      virtual bool GetBestCaptureFormat(const cricket::VideoFormat& desired,
					cricket::VideoFormat* best_format) OVERRIDE;
      virtual bool IsScreencast() const OVERRIDE;

    private:
      DISALLOW_COPY_AND_ASSIGN(RosVideoCapturer);
      image_transport::Subscriber sub;
    };


  class RosVideoCapturerFactory : public cricket::VideoDeviceCapturerFactory
  {
  public:
    RosVideoCapturerFactory() {}
    virtual ~RosVideoCapturerFactory() {}

    virtual cricket::VideoCapturer* Create(const cricket::Device& device) {
      return new RosVideoCapturer();
    }
  };


} // namespace scy


#endif
