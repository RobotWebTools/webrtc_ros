#include "webrtc_ros/ros_video_capturer.h"
#include "webrtc/base/bind.h"

namespace webrtc_ros
{


RosVideoCapturer::RosVideoCapturer(image_transport::ImageTransport it, const std::string& topic)
  : it_(it), topic_(topic), start_thread_(nullptr)
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
}


cricket::CaptureState RosVideoCapturer::Start(const cricket::VideoFormat& capture_format)
{
  start_thread_ = rtc::Thread::Current();
  try
  {
    ROS_INFO("Starting ROS subscriber");
    if (capture_state() == cricket::CS_RUNNING)
    {
      ROS_WARN("Start called when it's already started.");
      return capture_state();
    }

    sub_ = it_.subscribe(topic_, 1, &RosVideoCapturer::imageCallback, this);

    SetCaptureFormat(&capture_format);
    return cricket::CS_RUNNING;
  }
  catch (...) {}
  return cricket::CS_FAILED;
}


void RosVideoCapturer::Stop()
{
  start_thread_ = nullptr;
  try
  {
    ROS_INFO("Stopping ROS subscriber");
    if (capture_state() == cricket::CS_STOPPED)
    {
      ROS_WARN("Stop called when it's already stopped.");
      return;
    }
    sub_.shutdown();
    SetCaptureFormat(NULL);
    SetCaptureState(cricket::CS_STOPPED);
  }
  catch (...) {}
}


void RosVideoCapturer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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
    bgr = cv::Mat(bgr.rows, bgr.cols, CV_8UC3);
    cv::cvtColor(orig, bgr, CV_GRAY2BGR);
  }
  else
  {
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }

  cv::Mat yuv(bgr.rows, bgr.cols, CV_8UC4);
  cv::cvtColor(bgr, yuv, CV_BGR2YUV_I420);

  cricket::CapturedFrame frame;
  frame.width = bgr.cols;
  frame.height = bgr.rows;
  frame.fourcc = cricket::FOURCC_I420;
  frame.data_size = yuv.rows * yuv.step;
  frame.data = yuv.data;

  // This must be invoked on the worker thread, which should be the thread start was called on
  // This code is based on talk/media/webrtc/webrtcvideocapturer.cc, WebRtcVideoCapturer::OnIncomingCapturedFrame
  if (start_thread_->IsCurrent()) {
    SignalFrameCapturedOnStartThread(&frame);
  } else {
    start_thread_->Invoke<void>(rtc::Bind(&RosVideoCapturer::SignalFrameCapturedOnStartThread,
					  this, &frame));
  }
}

void RosVideoCapturer::SignalFrameCapturedOnStartThread(cricket::CapturedFrame *frame) {
  SignalFrameCaptured(this, frame);
}

bool RosVideoCapturer::IsRunning()
{
  return capture_state() == cricket::CS_RUNNING;
}


bool RosVideoCapturer::GetPreferredFourccs(std::vector<uint32>* fourccs)
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


}
