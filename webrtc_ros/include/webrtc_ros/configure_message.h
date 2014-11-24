#ifndef WEBRTC_ROS_CONFIGURE_MESSAGE_H_
#define WEBRTC_ROS_CONFIGURE_MESSAGE_H_

#include "webrtc_ros/webrtc_ros_message.h"
#include "webrtc/base/json.h"
#include "talk/app/webrtc/jsep.h"

namespace webrtc_ros {

class ConfigureMessage
{
 public:
  static std::string kSubscribedVideoTopicFieldName;
  static std::string kSubscribedAudioTopicFieldName;
  static std::string kPublishedVideoTopicFieldName;
  static std::string kPublishedAudioTopicFieldName;
  static std::string kConfigureType;

  static bool isConfigure(const Json::Value& message_json);

  bool fromJson(const Json::Value& message_json);
  std::string toJson();

  ConfigureMessage();

  std::string subscribed_video_topic;
  std::string subscribed_audio_topic;
  std::string published_video_topic;
  std::string published_audio_topic;
};

}

#endif
