#ifndef WEBRTC_ROS_ICE_CANDIDATE_MESSAGE_H_
#define WEBRTC_ROS_ICE_CANDIDATE_MESSAGE_H_

#include <webrtc_ros/webrtc_ros_message.h>
#include <json/json.h>
#include <webrtc/api/jsep.h>


namespace webrtc_ros
{

class IceCandidateMessage
{
public:
  static std::string kIceCandidateType;
  static std::string kSdpMidFieldName;
  static std::string kSdpMlineIndexFieldName;
  static std::string kCandidateFieldName;

  static bool isIceCandidate(const Json::Value& message_json);

  bool fromJson(const Json::Value& message_json);
  bool fromIceCandidate(const webrtc::IceCandidateInterface& ice_candidate);

  webrtc::IceCandidateInterface* createIceCandidate();
  std::string toJson();

  IceCandidateMessage();

  std::string sdp_mid;
  int sdp_mline_index;
  std::string candidate;

};

}

#endif
