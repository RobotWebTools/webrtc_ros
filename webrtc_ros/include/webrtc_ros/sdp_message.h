#ifndef WEBRTC_ROS_SDP_MESSAGE_H_
#define WEBRTC_ROS_SDP_MESSAGE_H_

#include <webrtc_ros/webrtc_ros_message.h>
#include <json/json.h>
#include <webrtc/api/jsep.h>


namespace webrtc_ros
{

class SdpMessage
{
public:
  static std::string kSdpFieldName;
  static std::string kSdpOfferType;
  static std::string kSdpAnswerType;

  static bool isSdpOffer(const Json::Value& message_json);
  static bool isSdpAnswer(const Json::Value& message_json);

  bool fromJson(const Json::Value& message_json);
  bool fromSessionDescription(const webrtc::SessionDescriptionInterface& description);

  webrtc::SessionDescriptionInterface* createSessionDescription();
  std::string toJson();

  SdpMessage();

  std::string type;
  std::string sdp;

};

}

#endif
