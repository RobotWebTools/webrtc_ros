#ifndef WEBRTC_ROS_ROS_LOG_CONTEXT_H_
#define WEBRTC_ROS_ROS_LOG_CONTEXT_H_

#include <webrtc/system_wrappers/include/trace.h>
#include <webrtc/base/logging.h>
#include <webrtc/base/stream.h>
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
class RosLogContext : public webrtc::TraceCallback, public rtc::LogSink {
 public:
  virtual ~RosLogContext();

  // webrtc::TraceCallback
  virtual void Print(webrtc::TraceLevel level, const char* message, int length) override;
  // rtc::LogSink
  virtual void OnLogMessage (const std::string& message) override;

private:
  RosLogContext();

  rtc::LoggingSeverity old_log_to_debug_;

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
