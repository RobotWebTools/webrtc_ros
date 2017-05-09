#include "webrtc_ros/ros_log_context.h"
#include <boost/algorithm/string/trim.hpp>
#include <ros/ros.h>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

namespace webrtc_ros {

RosLogContext::RosLogContext() {
  webrtc::Trace::CreateTrace();
  if (webrtc::Trace::SetTraceCallback(this) != 0)
    ROS_FATAL_NAMED("webrtc", "Failed to enable webrtc ROS trace context");
  rtc::LogMessage::AddLogToStream(this, rtc::LS_INFO);

  // this disables logging to stderr (which is done by default)
  old_log_to_debug_ = rtc::LogMessage::GetLogToDebug();
  rtc::LogMessage::LogToDebug(rtc::LS_NONE);
}

RosLogContext::~RosLogContext() {
  rtc::LogMessage::LogToDebug(old_log_to_debug_);
  rtc::LogMessage::RemoveLogToStream(this);
  if (webrtc::Trace::SetTraceCallback(NULL) != 0)
    ROS_FATAL_NAMED("webrtc", "Failed to disable webrtc ROS trace context");
  webrtc::Trace::ReturnTrace();
}

/**
 * Function for simplifying creating raw log messages.
 * As of jade it is not possible to use the public APIs to do this, so we have
 * to do some hacks to make it work.
 */
static void CustomRosLog(ros::console::levels::Level level, const std::string& message, const std::string& file, int line, const std::string& function) {
  std::stringstream ss;
  ss << message;

  ROSCONSOLE_DEFINE_LOCATION(true, level, std::string(ROSCONSOLE_NAME_PREFIX) + ".webrtc");
#if ROS_VERSION_MINIMUM(1, 11, 0) // Indigo & Jade
  if (__rosconsole_define_location__enabled) {
    ros::console::print(0, __rosconsole_define_location__loc.logger_,
			level, ss, file.c_str(), line, function.c_str());
  }
#elif ROS_VERSION_MINIMUM(1, 10, 0) // Hydro
  if (enabled) {
    ros::console::print(0, loc.logger_,
			level, ss, file.c_str(), line, function.c_str());
  }
#else
    ROS_INFO_STREAM(message);
#endif
}


// webrtc::TraceCallback
static ros::console::levels::Level RosLogLevelFromWebRtcTraceLevel(webrtc::TraceLevel webrtc_level) {
  switch (webrtc_level) {
    case webrtc::kTraceStateInfo: return ::ros::console::levels::Debug;
    case webrtc::kTraceWarning: return ::ros::console::levels::Warn;
    case webrtc::kTraceError: return ::ros::console::levels::Error;
    case webrtc::kTraceCritical: return ::ros::console::levels::Fatal;
    case webrtc::kTraceApiCall: return ::ros::console::levels::Debug;//verbose
    case webrtc::kTraceModuleCall: return ::ros::console::levels::Debug;//verbose
    case webrtc::kTraceMemory: return ::ros::console::levels::Debug;//verbose
    case webrtc::kTraceTimer: return ::ros::console::levels::Debug;//verbose
    case webrtc::kTraceStream: return ::ros::console::levels::Debug;//verbose
    case webrtc::kTraceDebug: return ::ros::console::levels::Debug;
    case webrtc::kTraceInfo: return ::ros::console::levels::Debug;
    case webrtc::kTraceTerseInfo: return ::ros::console::levels::Info;
    default:
      ROS_WARN_STREAM("Unexpected webrtc::TraceLevel: " << webrtc_level);
      return ::ros::console::levels::Error;
  }
}

void RosLogContext::Print(webrtc::TraceLevel level, const char* message, int length) {
  CustomRosLog(RosLogLevelFromWebRtcTraceLevel(level), std::string(message, length), "", -1, "");
}


// rtc::StreamInterface
static boost::regex log_context_regex("^(\\w*)\\(([^:]+):(\\d+)\\): (.*)", boost::regex_constants::ECMAScript|boost::regex_constants::icase|boost::regex_constants::optimize);
static bool ParseLogMessageContext(const std::string message, rtc::LoggingSeverity* severity, std::string* file, int* line, std::string* actual_message) {
  boost::smatch match;
  if (boost::regex_match(message, match, log_context_regex)) {
    if(match.size() != 5)
      return false;
    std::string severity_str = match[1].str();
    if(severity_str.empty())
      *severity = rtc::LS_INFO;  // sensible default
    else if(severity_str == "Sensitive")
      *severity = rtc::LS_SENSITIVE;
    else if(severity_str == "Verbose")
      *severity = rtc::LS_VERBOSE;
    else if(severity_str == "Info")
      *severity = rtc::LS_INFO;
    else if(severity_str == "Warning")
      *severity = rtc::LS_WARNING;
    else if(severity_str == "Error")
      *severity = rtc::LS_ERROR;
    else
      return false;
    *file = match[2].str();
    std::string line_str = match[3].str();
    try {
      *line = boost::lexical_cast<int>(line_str);
    } catch(boost::bad_lexical_cast const&) {
      return false;
    }
    *actual_message = match[4].str();
    return true;
  }
  else {
    return false;
  }
}
static ::ros::console::levels::Level RosLogLevelFromRtcLoggingSeverity(rtc::LoggingSeverity severity) {
  switch (severity) {
  case rtc::LS_SENSITIVE: return ::ros::console::levels::Debug;
  case rtc::LS_VERBOSE:   return ::ros::console::levels::Debug;
  case rtc::LS_INFO:      return ::ros::console::levels::Debug; // This is too verbose to be info
  case rtc::LS_WARNING:   return ::ros::console::levels::Warn;
  case rtc::LS_ERROR:     return ::ros::console::levels::Error;
  default:
    ROS_WARN_STREAM("Unexpected rtc::LoggingSeverity: " << severity);
    return ::ros::console::levels::Error;
  }
}


void RosLogContext::OnLogMessage(const std::string& message)
{
  std::string trimmed_message = boost::algorithm::trim_copy(message);

  rtc::LoggingSeverity severity;
  std::string file;
  int line;
  std::string actual_message;
  if(ParseLogMessageContext(trimmed_message, &severity, &file, &line, &actual_message)) {
    CustomRosLog(RosLogLevelFromRtcLoggingSeverity(severity), actual_message, file, line, "");
  }
  else {
    ROS_WARN_STREAM_THROTTLE(10.0, "Failed to parse webrtc log message: " << message );
  }
}


unsigned int RosLogContextRef::usage_count = 0;
std::mutex RosLogContextRef::mutex;
RosLogContext *RosLogContextRef::context = nullptr;

RosLogContextRef::RosLogContextRef() {
  std::lock_guard<std::mutex> lock (mutex);
  if(context == nullptr) {
    context = new RosLogContext();
  }
  ++usage_count;
}
RosLogContextRef::~RosLogContextRef() {
  std::lock_guard<std::mutex> lock (mutex);
  --usage_count;
  if(usage_count == 0) {
    delete context;
    context = nullptr;
  }
}


}
