#include "webrtc_ros/ice_candidate_message.h"

namespace webrtc_ros
{

bool IceCandidateMessage::isIceCandidate(const Json::Value& message_json)
{
  return WebrtcRosMessage::isType(message_json, kIceCandidateType);
}

bool IceCandidateMessage::fromJson(const Json::Value& message_json)
{
  if (isIceCandidate(message_json))
  {
    if (!rtc::GetStringFromJsonObject(message_json, kSdpMidFieldName, &sdp_mid))
      return false;
    if (!rtc::GetIntFromJsonObject(message_json, kSdpMlineIndexFieldName, &sdp_mline_index))
      return false;
    if (!rtc::GetStringFromJsonObject(message_json, kCandidateFieldName, &candidate))
      return false;
    return true;
  }
  else
    return false;
}
bool IceCandidateMessage::fromIceCandidate(const webrtc::IceCandidateInterface& ice_candidate)
{
  sdp_mid = ice_candidate.sdp_mid();
  sdp_mline_index = ice_candidate.sdp_mline_index();
  if (!ice_candidate.ToString(&candidate))
  {
    return false;
  }
  return true;
}


webrtc::IceCandidateInterface* IceCandidateMessage::createIceCandidate()
{
  return webrtc::CreateIceCandidate(sdp_mid, sdp_mline_index, candidate, 0);
}

std::string IceCandidateMessage::toJson()
{
  Json::FastWriter writer;
  Json::Value message_json;

  message_json[WebrtcRosMessage::kMessageTypeFieldName] = kIceCandidateType;
  message_json[kSdpMidFieldName] = sdp_mid;
  message_json[kSdpMlineIndexFieldName] = sdp_mline_index;
  message_json[kCandidateFieldName] = candidate;
  return writer.write(message_json);
}


IceCandidateMessage::IceCandidateMessage() {}

std::string IceCandidateMessage::kIceCandidateType = "ice_candidate";
std::string IceCandidateMessage::kSdpMidFieldName = "sdp_mid";
std::string IceCandidateMessage::kSdpMlineIndexFieldName = "sdp_mline_index";
std::string IceCandidateMessage::kCandidateFieldName = "candidate";

}


