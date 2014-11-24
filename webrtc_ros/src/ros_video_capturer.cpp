#include "webrtc_ros/ros_video_capturer.h"

namespace webrtc_ros {


  RosVideoCapturer::RosVideoCapturer(image_transport::ImageTransport it, const std::string& topic)
    : it_(it), topic_(topic) {

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
    try {
      ROS_INFO("Starting ROS subscriber");
      if (capture_state() == cricket::CS_RUNNING) {
	ROS_WARN("Start called when it's already started.");
	return capture_state();
      }

      sub_ = it_.subscribe(topic_, 1, &RosVideoCapturer::imageCallback, this);

      SetCaptureFormat(&capture_format);
      return cricket::CS_RUNNING;
    } catch (...) {}
    return cricket::CS_FAILED;
  }


  void RosVideoCapturer::Stop()
  {
    try {
      ROS_INFO("Stopping ROS subscriber");
      if (capture_state() == cricket::CS_STOPPED) {
	ROS_WARN("Stop called when it's already stopped.");
	return;
      }
      sub_.shutdown();
      SetCaptureFormat(NULL);
      SetCaptureState(cricket::CS_STOPPED);
    } catch (...) {}
  }


  void RosVideoCapturer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat orig = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat yuv(orig.rows, orig.cols, CV_8UC4);
    cv::cvtColor(orig, yuv, CV_BGR2YUV_I420);

    cricket::CapturedFrame frame;
    frame.width = orig.cols;
    frame.height = orig.rows;
    frame.fourcc = cricket::FOURCC_I420;
    frame.data_size = yuv.rows * yuv.step;
    frame.data = yuv.data;

    SignalFrameCaptured(this, &frame);
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
