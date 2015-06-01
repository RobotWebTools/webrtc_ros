#ifndef WEBRTC_ROS_ROS_LOG_CONTEXT_H_
#define WEBRTC_ROS_ROS_LOG_CONTEXT_H_

#include "webrtc/system_wrappers/interface/trace.h"
#include "webrtc/base/logging.h"
#include "webrtc/base/stream.h"

namespace webrtc_ros
{

class RosLogContext : public webrtc::TraceCallback, public rtc::StreamInterface {
 public:
  RosLogContext(bool disable_log_to_debug);
  virtual ~RosLogContext();

  // webrtc::TraceCallback
  virtual void Print(webrtc::TraceLevel level, const char* message, int length);

  // rtc::StreamInterface
  virtual rtc::StreamState GetState() const;
  virtual rtc::StreamResult Read(void* buffer, size_t buffer_len,
				 size_t* read, int* error);
  virtual rtc::StreamResult Write(const void* data, size_t data_len,
				  size_t* written, int* error);
  virtual void Close();

private:
  int old_log_to_debug_;
  bool disabled_debug_;
};

}

#endif
