#include "webrtc_ros/configure_message.h"

namespace webrtc_ros {

bool ConfigureMessage::isConfigure(const Json::Value& message_json) {
  return WebrtcRosMessage::isType(message_json, kConfigureType);
}

bool ConfigureMessage::fromJson(const Json::Value& message_json) {
  if(isConfigure(message_json)){
    if(!GetStringFromJsonObject(message_json, kSubscribedVideoTopicFieldName, &subscribed_video_topic))
      subscribed_video_topic = "";
    if(!GetStringFromJsonObject(message_json, kSubscribedAudioTopicFieldName, &subscribed_audio_topic))
      subscribed_audio_topic = "";
    if(!GetStringFromJsonObject(message_json, kPublishedVideoTopicFieldName, &published_video_topic))
      published_video_topic = "";
    if(!GetStringFromJsonObject(message_json, kPublishedAudioTopicFieldName, &published_audio_topic))
      published_audio_topic = "";
    return true;
  }
  else
    return false;
}

std::string ConfigureMessage::toJson() {
  Json::FastWriter writer;
  Json::Value message_json;
  message_json[kSubscribedVideoTopicFieldName] = subscribed_video_topic;
  message_json[kSubscribedAudioTopicFieldName] = subscribed_audio_topic;
  message_json[kPublishedVideoTopicFieldName] = published_video_topic;
  message_json[kPublishedAudioTopicFieldName] = published_audio_topic;
  return writer.write(message_json);
}

ConfigureMessage::ConfigureMessage() {}

std::string ConfigureMessage::kSubscribedVideoTopicFieldName = "subscribed_video_topic";
std::string ConfigureMessage::kSubscribedAudioTopicFieldName = "subscribed_audio_topic";
std::string ConfigureMessage::kPublishedVideoTopicFieldName = "published_video_topic";
std::string ConfigureMessage::kPublishedAudioTopicFieldName = "published_audio_topic";
std::string ConfigureMessage::kConfigureType = "configure";

}


