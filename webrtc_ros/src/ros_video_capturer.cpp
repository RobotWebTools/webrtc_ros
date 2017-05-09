#include "webrtc_ros/ros_video_capturer.h"
#include "webrtc/base/bind.h"

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/enable_shared_from_this.hpp>

namespace webrtc_ros
{


RosVideoCapturer::RosVideoCapturer(image_transport::ImageTransport it, const std::string& topic)
  : start_thread_(nullptr), handler_(this), impl_(new RosVideoCapturerImpl(it, topic))
{

  // Default supported formats. Use ResetSupportedFormats to over write.
  std::vector<cricket::VideoFormat> formats;
  formats.push_back(cricket::VideoFormat(1280, 720,
                                         cricket::VideoFormat::FpsToInterval(30), cricket::FOURCC_I420));
  formats.push_back(cricket::VideoFormat(640, 480,
                                         cricket::VideoFormat::FpsToInterval(30), cricket::FOURCC_I420));
  formats.push_back(cricket::VideoFormat(320, 240,
                                         cricket::VideoFormat::FpsToInterval(30), cricket::FOURCC_I420));
  formats.push_back(cricket::VideoFormat(160, 120,
                                         cricket::VideoFormat::FpsToInterval(30), cricket::FOURCC_I420));
}


RosVideoCapturer::~RosVideoCapturer()
{
  Stop(); // Make sure were stopped so callbacks stop
}


cricket::CaptureState RosVideoCapturer::Start(const cricket::VideoFormat& capture_format)
{
  start_thread_ = rtc::Thread::Current();
  if (capture_state() == cricket::CS_RUNNING) {
    ROS_WARN("Start called when it's already started.");
    return capture_state();
  }

  impl_->Start(this);

  SetCaptureFormat(&capture_format);
  return cricket::CS_RUNNING;
}


void RosVideoCapturer::Stop()
{
  impl_->Stop();
  start_thread_ = nullptr;
  SetCaptureFormat(NULL);
  SetCaptureState(cricket::CS_STOPPED);
}

void RosVideoCapturer::imageCallback(const std::shared_ptr<webrtc::VideoFrame>& frame)
{
  // This must be invoked on the worker thread, which should be the thread start was called on
  // This code is based on talk/media/webrtc/webrtcvideocapturer.cc, WebRtcVideoCapturer::OnIncomingCapturedFrame
  if (start_thread_->IsCurrent()) {
    SignalFrameCapturedOnStartThread(frame);
  } else {
    // Cannot use invoke here because it can sometimes deadlock if the start thread is quit
    // before we reach this point
    start_thread_->Send(RTC_FROM_HERE, &handler_, 0, rtc::WrapMessageData<std::shared_ptr<webrtc::VideoFrame>>(frame));
  }
}

ImageMessageHandler::ImageMessageHandler(RosVideoCapturer *capturer) : capturer_(capturer) {}
void ImageMessageHandler::OnMessage(rtc::Message* msg)
{
  capturer_->SignalFrameCapturedOnStartThread(rtc::UseMessageData<std::shared_ptr<webrtc::VideoFrame>>(msg->pdata));
  delete msg->pdata; msg->pdata = nullptr;
}

void RosVideoCapturer::SignalFrameCapturedOnStartThread(const std::shared_ptr<webrtc::VideoFrame>& frame) {
  OnFrame(*frame, frame->width(), frame->height());
}

bool RosVideoCapturer::IsRunning()
{
  return capture_state() == cricket::CS_RUNNING;
}


bool RosVideoCapturer::GetPreferredFourccs(std::vector<uint32_t>* fourccs)
{
  if (!fourccs)
    return false;
  fourccs->push_back(cricket::FOURCC_I420);
  return true;
}


bool RosVideoCapturer::GetBestCaptureFormat(const cricket::VideoFormat& desired, cricket::VideoFormat* best_format)
{
  if (!best_format)
    return false;

  best_format->width = desired.width;
  best_format->height = desired.height;
  best_format->fourcc = cricket::FOURCC_I420;
  best_format->interval = desired.interval;
  return true;
}


bool RosVideoCapturer::IsScreencast() const
{
  return false;
}




RosVideoCapturerImpl::RosVideoCapturerImpl(image_transport::ImageTransport it, const std::string& topic)
  : it_(it), topic_(topic), capturer_(nullptr) {}

RosVideoCapturerImpl::~RosVideoCapturerImpl() {}


void RosVideoCapturerImpl::Start(RosVideoCapturer *capturer)
{
  std::unique_lock<std::mutex> lock(state_mutex_);

  ROS_INFO("Starting ROS subscriber");

  sub_ = it_.subscribe(topic_, 1, boost::bind(&RosVideoCapturerImpl::imageCallback, shared_from_this(), _1));
  capturer_ = capturer;
}


void RosVideoCapturerImpl::Stop()
{
  // Make sure to do this before aquiring lock so we don't deadlock with callback
  // This needs to aquire a lock that is heald which callbacks are dispatched
  if(sub_)
    sub_.shutdown();

  std::unique_lock<std::mutex> lock(state_mutex_);
  if(capturer_ == nullptr)
    return;

  ROS_INFO("Stopping ROS subscriber");

  capturer_ = nullptr;
}


void RosVideoCapturerImpl::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  if(capturer_ == nullptr)
    return;
  cv::Mat bgr;
  if (msg->encoding.find("F") != std::string::npos)
  {
    // scale floating point images
    cv::Mat float_image_bridge = cv_bridge::toCvCopy(msg, msg->encoding)->image;
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
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }

  cv::Mat yuv;
  cv::cvtColor(bgr, yuv, CV_BGR2YUV_I420);

  uint8_t* y = yuv.data;
  uint8_t* u = y + (bgr.cols * bgr.rows);
  uint8_t* v = u + (bgr.cols * bgr.rows) / 4;

  std::shared_ptr<webrtc::VideoFrame> frame = std::make_shared<webrtc::VideoFrame>(
      webrtc::I420Buffer::Copy(bgr.cols, bgr.rows, y, bgr.cols, u, bgr.cols / 2, v, bgr.cols / 2),
      msg->header.stamp.toNSec() / 1000000,
      msg->header.stamp.toNSec() / 1000000,
      webrtc::kVideoRotation_0
  );
  capturer_->imageCallback(frame);
}

}

