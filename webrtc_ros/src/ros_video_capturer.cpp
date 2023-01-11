#include "webrtc_ros/ros_video_capturer.h"
#include "webrtc/rtc_base/bind.h"

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/enable_shared_from_this.hpp>

namespace webrtc_ros
{


RosVideoCapturer::RosVideoCapturer(const ImageTransportFactory& it, const std::string& topic, const std::string& transport)
  : impl_(new RosVideoCapturerImpl(it, topic, transport))
{
}


RosVideoCapturer::~RosVideoCapturer()
{
  Stop(); // Make sure were stopped so callbacks stop
}


void RosVideoCapturer::Start()
{
  impl_->Start(this);
}

void RosVideoCapturer::Stop()
{
  impl_->Stop();
}

void RosVideoCapturer::imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg)
{
  cv::Mat bgr;
  if (msg->encoding.find("F") != std::string::npos)
  {
    // scale floating point images
    cv::Mat float_image_bridge = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    double max_val;
    cv::minMaxIdx(float_image, 0, &max_val);

    if (max_val > 0)
    {
      float_image *= (255 / max_val);
    }
    cv::Mat orig;
    float_image.convertTo(orig, CV_8U);
    cv::cvtColor(orig, bgr, CV_GRAY2BGR);
  }
  else
  {
    bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  int64_t camera_time_us = msg->header.stamp.nanosec / 1000;
  int64_t system_time_us = rclcpp::Clock().now().nanoseconds() / 1000;
  cv::Rect roi;
  int out_width, out_height;
  int64_t translated_camera_time_us;
  if (AdaptFrame(bgr.cols, bgr.rows, system_time_us, &out_width, &out_height, &roi.width, &roi.height, &roi.x, &roi.y))
  {
    cv::Mat yuv;
    if (out_width == roi.width && out_height == roi.height)
    {
      cv::cvtColor(bgr(roi), yuv, CV_BGR2YUV_I420);
    }
    else
    {
      cv::Mat m;
      cv::resize(bgr(roi), m, cv::Size2i(out_width, out_height), 0, 0, out_width < roi.width ? cv::INTER_AREA : cv::INTER_LINEAR);
      cv::cvtColor(m, yuv, CV_BGR2YUV_I420);
    }
    uint8_t* y = yuv.data;
    uint8_t* u = y + (out_width * out_height);
    uint8_t* v = u + (out_width * out_height) / 4;

    std::shared_ptr<webrtc::VideoFrame> frame = std::make_shared<webrtc::VideoFrame>(
        webrtc::I420Buffer::Copy(out_width, out_height, y, out_width, u, out_width / 2, v, out_width / 2),
        webrtc::kVideoRotation_0,
        system_time_us
    );
    // Apparently, the OnFrame() method could not be called from arbitrary threads in ancient times, and there
    // used to be all kinds of shenanigans here to make sure it is called from the original thread, causing
    // a subtle deadlock bug on object destruction.
    //
    // So I decided to be blunt and just call it like it is:
    OnFrame(*frame);
  }
}



bool RosVideoCapturer::is_screencast() const
{
  return false;
}

 
absl::optional<bool> RosVideoCapturer::needs_denoising() const
{
	return false;
}
webrtc::MediaSourceInterface::SourceState RosVideoCapturer::state() const
{
	return webrtc::MediaSourceInterface::kLive;
}
bool RosVideoCapturer::remote() const
{
	return false;
}



RosVideoCapturerImpl::RosVideoCapturerImpl(const ImageTransportFactory& it, const std::string& topic, const std::string& transport)
  : it_(it), topic_(topic), transport_(transport), capturer_(nullptr) {}

void RosVideoCapturerImpl::Start(RosVideoCapturer *capturer)
{
  std::unique_lock<std::mutex> lock(state_mutex_);

  sub_ = it_.subscribe(topic_, std::bind(&RosVideoCapturerImpl::imageCallback, shared_from_this(), std::placeholders::_1), transport_);
  capturer_ = capturer;
}


void RosVideoCapturerImpl::Stop()
{
  // Make sure to do this before aquiring lock so we don't deadlock with callback
  // This needs to aquire a lock that is heald which callbacks are dispatched
  sub_.shutdown();

  std::unique_lock<std::mutex> lock(state_mutex_);
  if(capturer_ == nullptr)
    return;

  capturer_ = nullptr;
}


void RosVideoCapturerImpl::imageCallback(const sensor_msgs::msg::Image::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  if(capturer_ == nullptr)
    return;
  capturer_->imageCallback(msg);
}

}

