#include "talk/media/base/mediacommon.h"
#include "webrtc_ros/ros_media_device_manager.h"
#include "webrtc_ros/ros_video_capturer.h"
#include "webrtc_ros/ros_video_renderer.h"

namespace webrtc_ros {

RosMediaDeviceManager::RosMediaDeviceManager(image_transport::ImageTransport it) : it_(it) {}
RosMediaDeviceManager::~RosMediaDeviceManager() {}

bool RosMediaDeviceManager::Init() {
  return true;
}
void RosMediaDeviceManager::Terminate() {
}

int RosMediaDeviceManager::GetCapabilities() {
  return cricket::VIDEO_RECV | cricket::VIDEO_SEND; // AUDIO_SEND | AUDIO_RECV
}

bool RosMediaDeviceManager::GetAudioInputDevices(std::vector<cricket::Device>* devices) {
  //TODO: implement
  return false;
}
bool RosMediaDeviceManager::GetAudioOutputDevices(std::vector<cricket::Device>* devices) {
  //TODO: implement
  return false;
}

bool RosMediaDeviceManager::GetAudioInputDevice(const std::string& name, cricket::Device* out) {
  //TODO: implement
  return false;
}
bool RosMediaDeviceManager::GetAudioOutputDevice(const std::string& name, cricket::Device* out) {
  //TODO: implement
  return false;
}

bool RosMediaDeviceManager::GetVideoCaptureDevices(std::vector<cricket::Device>* devs) {
  //TODO: get list of image topics
  devs->clear();
  return true;
}
bool RosMediaDeviceManager::GetVideoOutputDevices(std::vector<cricket::Device>* devs) {
  devs->clear();
  return true;
}

bool RosMediaDeviceManager::GetVideoCaptureDevice(const std::string& name, cricket::Device* out) {
  out->name = name;
  out->id = name;
  return true;
}
bool RosMediaDeviceManager::GetVideoOutputDevice(const std::string& name, cricket::Device* out) {
  out->name = name;
  out->id = name;
  return false;
}

cricket::VideoCapturer* RosMediaDeviceManager::CreateVideoCapturer(const cricket::Device& device) const {
  return new RosVideoCapturer(it_, device.id);
}

RosVideoRenderer* RosMediaDeviceManager::CreateVideoRenderer(const cricket::Device& device) const {
  return new RosVideoRenderer(it_, device.id);
}




void RosMediaDeviceManager::SetVideoDeviceCapturerFactory(cricket::VideoDeviceCapturerFactory* video_device_capturer_factory) {
  // ignore
  if(video_device_capturer_factory)
    delete video_device_capturer_factory;
}
void RosMediaDeviceManager::SetScreenCapturerFactory(cricket::ScreenCapturerFactory* screen_capturer_factory) {
  // ignore
  if(screen_capturer_factory)
    delete screen_capturer_factory;
}

void RosMediaDeviceManager::SetVideoCaptureDeviceMaxFormat(const std::string& usb_id,
							   const cricket::VideoFormat& max_format){
  // ignore
}
void RosMediaDeviceManager::ClearVideoCaptureDeviceMaxFormat(const std::string& usb_id) {
  // ignore
}


bool RosMediaDeviceManager::GetWindows(std::vector<rtc::WindowDescription>* descriptions){
  // ignore
  return false;
}
bool RosMediaDeviceManager::GetDesktops(std::vector<rtc::DesktopDescription>* descriptions){
  // ignore
  return false;
}
cricket::VideoCapturer* RosMediaDeviceManager::CreateScreenCapturer(const cricket::ScreencastId& screenid) const{
  // ignore
  return NULL;
}


}
