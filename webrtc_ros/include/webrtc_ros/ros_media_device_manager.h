#ifndef WEBRTC_ROS_ROS_MEDIA_DEVICE_MANAGER_H_
#define WEBRTC_ROS_ROS_MEDIA_DEVICE_MANAGER_H_

#include "talk/media/devices/devicemanager.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace webrtc_ros {

class RosMediaDeviceManager : public cricket::DeviceManagerInterface
{
 public:
  RosMediaDeviceManager(image_transport::ImageTransport it);
  virtual ~RosMediaDeviceManager();

  virtual bool Init();
  virtual void Terminate();

  virtual int GetCapabilities();

  virtual bool GetAudioInputDevices(std::vector<cricket::Device>* devices);
  virtual bool GetAudioOutputDevices(std::vector<cricket::Device>* devices);

  virtual bool GetAudioInputDevice(const std::string& name, cricket::Device* out);
  virtual bool GetAudioOutputDevice(const std::string& name, cricket::Device* out);

  virtual bool GetVideoCaptureDevices(std::vector<cricket::Device>* devs);
  virtual bool GetVideoOutputDevices(std::vector<cricket::Device>* devs);

  virtual bool GetVideoCaptureDevice(const std::string& name, cricket::Device* out);
  virtual bool GetVideoOutputDevice(const std::string& name, cricket::Device* out);

  virtual void SetVideoDeviceCapturerFactory(cricket::VideoDeviceCapturerFactory* video_device_capturer_factory);
  virtual void SetScreenCapturerFactory(cricket::ScreenCapturerFactory* screen_capturer_factory);

  virtual void SetVideoCaptureDeviceMaxFormat(const std::string& usb_id,
					      const cricket::VideoFormat& max_format);
  virtual void ClearVideoCaptureDeviceMaxFormat(const std::string& usb_id);

  virtual cricket::VideoCapturer* CreateVideoCapturer(const cricket::Device& device) const;

  virtual bool GetWindows(std::vector<rtc::WindowDescription>* descriptions);
  virtual bool GetDesktops(std::vector<rtc::DesktopDescription>* descriptions);
  virtual cricket::VideoCapturer* CreateScreenCapturer(const cricket::ScreencastId& screenid) const;


 private:
  DISALLOW_COPY_AND_ASSIGN(RosMediaDeviceManager);

  image_transport::ImageTransport it_;
};



} // namespace scy


#endif
