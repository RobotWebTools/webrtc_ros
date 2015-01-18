#include "webrtc_ros/ros_trace_context.h"
#include <ros/ros.h>

namespace webrtc_ros {

static ::ros::console::levels::Level RosLogLevelFromWebRtcLogLevel(webrtc::TraceLevel webrtc_level) {
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
      ROS_WARN_STREAM_NAMED("webrtc", "Unexpected log level" << webrtc_level);
      return ::ros::console::levels::Fatal;
  }
}

RosTraceContext::RosTraceContext() {
  webrtc::Trace::CreateTrace();
  if (webrtc::Trace::SetTraceCallback(this) != 0)
    ROS_FATAL_NAMED("webrtc", "Failed to enable webrtc ROS trace context");
}

RosTraceContext::~RosTraceContext() {
  if (webrtc::Trace::SetTraceCallback(NULL) != 0)
    ROS_FATAL_NAMED("webrtc", "Failed to disable webrtc ROS trace context");
  webrtc::Trace::ReturnTrace();
}

void RosTraceContext::Print(webrtc::TraceLevel level, const char* message, int length) {
  ROS_LOG(RosLogLevelFromWebRtcLogLevel(level), std::string(ROSCONSOLE_NAME_PREFIX) + ".webrtc", "%.*s", length, message);
}

}
