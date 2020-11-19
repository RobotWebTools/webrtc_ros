#ifndef WEBRTC_ROS_CONFIGURE_MESSAGE_H_
#define WEBRTC_ROS_CONFIGURE_MESSAGE_H_

#include <webrtc_ros/webrtc_ros_message.h>
#include <json/json.h>
#include <webrtc/api/jsep.h>


namespace webrtc_ros
{

struct ConfigureAction
{
  bool fromJson(const Json::Value& action_json);
  void toJson(Json::Value* action_json) const;

  std::string type;
  std::map<std::string, std::string> properties;

  static std::string kTypeFieldName;

  static std::string kAddStreamActionName;
  static std::string kRemoveStreamActionName;
  static std::string kAddVideoTrackActionName;
  static std::string kAddAudioTrackActionName;
  static std::string kExpectStreamActionName;
  static std::string kExpectVideoTrackActionName;
};

class ConfigureMessage
{
public:
  static std::string kActionsFieldName;
  static std::string kConfigureType;

  static bool isConfigure(const Json::Value& message_json);

  bool fromJson(const Json::Value& message_json);
  std::string toJson() const;

  ConfigureMessage();

  std::vector<ConfigureAction> actions;
};

}

#endif
