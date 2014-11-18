#include "webrtc_ros/videocapturerocv.h"

using std::endl;


namespace scy {


  VideoCapturerOCV::VideoCapturerOCV(int deviceId)
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


  VideoCapturerOCV::~VideoCapturerOCV() 
  {
  }


  cricket::CaptureState VideoCapturerOCV::Start(const cricket::VideoFormat& capture_format)
  {
    try { 
      std::cout << "Start" << endl;
      if (capture_state() == cricket::CS_RUNNING) {
	std::cout << "Start called when it's already started." << endl;
	return capture_state();
      }

      // TODO: Honour VideoFormat

      ros::NodeHandle nh;
      image_transport::ImageTransport it(nh);
      sub = it.subscribe("image", 1, &VideoCapturerOCV::onFrameCaptured, this);

      SetCaptureFormat(&capture_format);
      return cricket::CS_RUNNING;
    } catch (...) {}
    return cricket::CS_FAILED;
  }


  void VideoCapturerOCV::Stop()
  {
    try { 
      std::cout << "Stop" << endl;
      if (capture_state() == cricket::CS_STOPPED) {
	std::cout << "Stop called when it's already stopped." << endl;
	return;
      }
      sub.shutdown();
      SetCaptureFormat(NULL);
      SetCaptureState(cricket::CS_STOPPED);
      return;
    } catch (...) {}
    return;
  }


  void VideoCapturerOCV::onFrameCaptured(const sensor_msgs::ImageConstPtr& msg) 
  {
    cv::Mat orig = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat yuv(orig.rows, orig.cols, CV_8UC4);
    cv::cvtColor(orig, yuv, CV_BGR2YUV_I420);

    ROS_INFO("Got frame %dx%d", orig.cols, orig.rows);

    cricket::CapturedFrame frame;
    frame.width = orig.cols;
    frame.height = orig.rows;
    frame.fourcc = cricket::FOURCC_I420;
    frame.data_size = yuv.rows * yuv.step;
    frame.data = yuv.data;

    SignalFrameCaptured(this, &frame);
  }


  bool VideoCapturerOCV::IsRunning()
  {
    return capture_state() == cricket::CS_RUNNING;
  }


  bool VideoCapturerOCV::GetPreferredFourccs(std::vector<uint32>* fourccs)
  {
    if (!fourccs)
      return false;
    fourccs->push_back(cricket::FOURCC_I420);
    return true;
  }


  bool VideoCapturerOCV::GetBestCaptureFormat(const cricket::VideoFormat& desired, cricket::VideoFormat* best_format)
  {
    if (!best_format)
      return false;

    // VideoCapturerOCV does not support capability enumeration.
    // Use the desired format as the best format.
    best_format->width = desired.width;
    best_format->height = desired.height;
    best_format->fourcc = cricket::FOURCC_I420;
    best_format->interval = desired.interval;
    return true;
  }


  bool VideoCapturerOCV::IsScreencast() const 
  {
    return false;
  }


} // namespace scy
