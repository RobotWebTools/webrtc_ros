#ifndef WEBRTC_ROS_ROS_VIDEO_RENDERER_H_
#define WEBRTC_ROS_ROS_VIDEO_RENDERER_H_

#include "talk/app/webrtc/mediastreaminterface.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

namespace webrtc_ros
{


class RosVideoRenderer :
  public webrtc::VideoRendererInterface
{
public:
  RosVideoRenderer(image_transport::ImageTransport it, const std::string& topic);
  virtual ~RosVideoRenderer();

  virtual void SetSize(int width, int height);
  virtual void RenderFrame(const cricket::VideoFrame* frame);

private:
  DISALLOW_COPY_AND_ASSIGN(RosVideoRenderer);

  image_transport::ImageTransport it_;
  const std::string topic_;
  image_transport::Publisher pub_;
};



}


#endif
