#ifndef WEBRTC_ROS_WEBRTC_ROS_MESSAGE_H_
#define WEBRTC_ROS_WEBRTC_ROS_MESSAGE_H_

#include <webrtc/base/json.h>

namespace webrtc_ros
{

class WebrtcRosMessage
{
public:
  static std::string kMessageTypeFieldName;

  static bool isType(const Json::Value& message_json, const std::string& type);
  static bool getType(const Json::Value& message_json, std::string* type);
};

}

#endif
