#ifndef WEBRTC_ROS_ROS_LOG_CONTEXT_H_
#define WEBRTC_ROS_ROS_LOG_CONTEXT_H_

#include "webrtc/system_wrappers/interface/trace.h"
#include "webrtc/base/logging.h"
#include "webrtc/base/stream.h"
#include <mutex>

namespace webrtc_ros
{

/**
 * Contains callbacks for the webrtc logging mechenisms that forward messages
 * to the ROS logging infrastructure. Webrtc logging is done globaly so only
 * one instance of this class ever exists at a time. Use RosLogContextRef to
 * indicate that the logging context should be kept alive. Logging hooks are
 * installed on construction and removed on destruction.
 */
class RosLogContext : public webrtc::TraceCallback, public rtc::StreamInterface {
 public:
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
  RosLogContext();

  int old_log_to_debug_;

  friend class RosLogContextRef;
};

/**
 * Represents a reference indicating that the webrtc ROS logging context should
 * be kept alive. When the first one is constructed the ROS log context will be
 * installed and when the last one is destroyed the ROS log context will be
 * uninstalled.
 * This construction/destruction is thread safe
 */
class RosLogContextRef {
public:
  RosLogContextRef();
  ~RosLogContextRef();
private:
  static unsigned int usage_count;
  static std::mutex mutex;
  static RosLogContext *context;
};

}

#endif
