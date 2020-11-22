#include "webrtc_ros/webrtc_ros_message.h"

namespace webrtc_ros
{

bool WebrtcRosMessage::isType(const Json::Value& message_json, const std::string& expected_type)
{
  std::string type;
  if (getType(message_json, &type))
    return expected_type.compare(type) == 0;
  return false;
}

bool WebrtcRosMessage::getType(const Json::Value& message_json, std::string* type)
{
  return WebrtcRosJsonParser::GetStringFromJsonObject(message_json, kMessageTypeFieldName, type);
}

std::string WebrtcRosMessage::kMessageTypeFieldName = "type";


}
