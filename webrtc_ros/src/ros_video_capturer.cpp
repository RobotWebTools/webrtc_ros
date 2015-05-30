#include "webrtc_ros/ros_video_capturer.h"
#include "webrtc/base/bind.h"

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

void RosVideoCapturer::imageCallback(cricket::CapturedFrame *frame)
{
  // This must be invoked on the worker thread, which should be the thread start was called on
  // This code is based on talk/media/webrtc/webrtcvideocapturer.cc, WebRtcVideoCapturer::OnIncomingCapturedFrame
  if (start_thread_->IsCurrent()) {
    SignalFrameCapturedOnStartThread(frame);
    delete frame;
  } else {
    // Cannot use invoke here because it can sometimes deadlock if the start thread is quit
    // before we reach this point
    start_thread_->Post(&handler_, 0, rtc::WrapMessageData(frame), true);
  }
}

ImageMessageHandler::ImageMessageHandler(RosVideoCapturer *capturer) : capturer_(capturer) {}
void ImageMessageHandler::OnMessage(rtc::Message* msg)
{
  capturer_->SignalFrameCapturedOnStartThread(rtc::UseMessageData<cricket::CapturedFrame*>(msg->pdata));
  delete rtc::UseMessageData<cricket::CapturedFrame*>(msg->pdata);
  delete msg->pdata;
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
    bgr = cv::Mat(bgr.rows, bgr.cols, CV_8UC3);
    cv::cvtColor(orig, bgr, CV_GRAY2BGR);
  }
  else
  {
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }

  cv::Mat yuv(bgr.rows, bgr.cols, CV_8UC4);
  cv::cvtColor(bgr, yuv, CV_BGR2YUV_I420);

  cricket::CapturedFrame *frame = new cricket::CapturedFrame;
  frame->width = bgr.cols;
  frame->height = bgr.rows;
  frame->fourcc = cricket::FOURCC_I420;
  frame->data_size = yuv.rows * yuv.step;
  frame->data = yuv.data;

  capturer_->imageCallback(frame);
}

}
