#ifndef WEBRTC_ROS_ROS_TRACE_CONTEXT_H_
#define WEBRTC_ROS_ROS_TRACE_CONTEXT_H_

#include "webrtc/system_wrappers/interface/trace.h"

namespace webrtc_ros
{

class RosTraceContext : public webrtc::TraceCallback {
 public:
  RosTraceContext();
  virtual ~RosTraceContext();

  virtual void Print(webrtc::TraceLevel level, const char* message, int length);
};


}

#endif
