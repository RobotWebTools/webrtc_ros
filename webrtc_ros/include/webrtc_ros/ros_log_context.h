#ifndef WEBRTC_ROS_ROS_LOG_CONTEXT_H_
#define WEBRTC_ROS_ROS_LOG_CONTEXT_H_

#include "webrtc/system_wrappers/interface/trace.h"
#include "webrtc/base/logging.h"
#include "webrtc/base/stream.h"

namespace webrtc_ros
{

class RosLogContext : public webrtc::TraceCallback {
 public:
  RosLogContext();
  virtual ~RosLogContext();

  virtual void Print(webrtc::TraceLevel level, const char* message, int length);
};


}

#endif
