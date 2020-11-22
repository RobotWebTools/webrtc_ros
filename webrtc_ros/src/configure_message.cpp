#include "webrtc_ros/configure_message.h"

namespace webrtc_ros
{

bool ConfigureAction::fromJson(const Json::Value& action_json)
{
  if (!WebrtcRosJsonParser::GetStringFromJsonObject(action_json, kTypeFieldName, &type))
    return false;
  properties.clear();
  for(auto itr = action_json.begin(); itr != action_json.end(); itr++)
  {
    if(itr.key() != kTypeFieldName)
    {
      properties[itr.key().asString()] = (*itr).asString();
    }
  }
  return true;
}
void ConfigureAction::toJson(Json::Value* action_json) const
{
  (*action_json)[kTypeFieldName] = type;
}

std::string ConfigureAction::kTypeFieldName = "type";

std::string ConfigureAction::kAddStreamActionName = "add_stream";
std::string ConfigureAction::kRemoveStreamActionName = "remove_stream";
std::string ConfigureAction::kAddVideoTrackActionName = "add_video_track";
std::string ConfigureAction::kAddAudioTrackActionName = "add_audio_track";
std::string ConfigureAction::kExpectStreamActionName = "expect_stream";
std::string ConfigureAction::kExpectVideoTrackActionName = "expect_video_track";

bool ConfigureMessage::isConfigure(const Json::Value& message_json)
{
  return WebrtcRosMessage::isType(message_json, kConfigureType);
}

bool ConfigureMessage::fromJson(const Json::Value& message_json)
{
  if (isConfigure(message_json))
  {
    Json::Value action_array_json = message_json[kActionsFieldName];
    if(!action_array_json.isArray())
      return false;

    actions.resize(action_array_json.size());
    for(unsigned i = 0; i < action_array_json.size(); ++i) {
      actions[i].fromJson(action_array_json[i]);
    }
    return true;
  }
  else
    return false;
}

std::string ConfigureMessage::toJson() const
{
  Json::FastWriter writer;
  Json::Value message_json;
  Json::Value action_array_json(Json::arrayValue);
  for(const ConfigureAction& action : actions) {
    Json::Value action_json;
    action.toJson(&action_json);
    action_array_json.append(action_json);
  }
  message_json[kActionsFieldName] = action_array_json;
  return writer.write(message_json);
}

ConfigureMessage::ConfigureMessage() {}

std::string ConfigureMessage::kActionsFieldName = "actions";
std::string ConfigureMessage::kConfigureType = "configure";

}


