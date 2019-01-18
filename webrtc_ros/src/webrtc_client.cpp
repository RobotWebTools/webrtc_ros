#include <ros/ros.h>
#include <webrtc_ros/webrtc_client.h>
#include <webrtc_ros/webrtc_ros_message.h>
#include <webrtc_ros/sdp_message.h>
#include <webrtc_ros/ice_candidate_message.h>
#include <webrtc/base/json.h>
//#include "talk/media/devices/devicemanager.h"
#include <webrtc/media/base/videosourceinterface.h>
#include <webrtc/base/bind.h>
#include <webrtc_ros/ros_video_capturer.h>

namespace webrtc_ros
{

WebrtcClientObserverProxy::WebrtcClientObserverProxy(WebrtcClientWeakPtr client_weak)
  : client_weak_(client_weak) {}

void WebrtcClientObserverProxy::OnSuccess(webrtc::SessionDescriptionInterface* description)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnSessionDescriptionSuccess(description);
}
void WebrtcClientObserverProxy::OnFailure(const std::string& error)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnSessionDescriptionFailure(error);
}
void WebrtcClientObserverProxy::OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnAddRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnRemoveRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface>)
{
}
void WebrtcClientObserverProxy::OnRenegotiationNeeded()
{
}
void WebrtcClientObserverProxy::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnIceCandidate(candidate);
}
void WebrtcClientObserverProxy::OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState)
{
}
void WebrtcClientObserverProxy::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState)
{
}
void WebrtcClientObserverProxy::OnIceCandidatesRemoved(const std::vector<cricket::Candidate>&)
{
}
void WebrtcClientObserverProxy::OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState)
{
}


WebrtcClient::WebrtcClient(ros::NodeHandle& nh, const ImageTransportFactory& itf, const std::string& transport, SignalingChannel* signaling_channel)
  : nh_(nh), it_(nh_), itf_(itf), transport_(transport), signaling_channel_(signaling_channel),
    signaling_thread_(rtc::Thread::Current())
{
  ROS_INFO("Creating WebrtcClient");
  peer_connection_factory_  = webrtc::CreatePeerConnectionFactory();
  if (!peer_connection_factory_.get())
  {
    ROS_WARN("Could not create peer connection factory");
    invalidate();
    return;
  }
  peer_connection_constraints_.SetAllowDtlsSctpDataChannels();
  media_constraints_.AddOptional(webrtc::MediaConstraintsInterface::kOfferToReceiveAudio, true);
  media_constraints_.AddOptional(webrtc::MediaConstraintsInterface::kOfferToReceiveVideo, true);
  ping_timer_ = nh_.createTimer(ros::Duration(10.0), boost::bind(&WebrtcClient::ping_timer_callback, this, _1));
}
WebrtcClient::~WebrtcClient()
{
  if(valid()) {
    ROS_FATAL("WebrtcClient destructor should only be called once it's invalidated");
  }
  ROS_INFO("Destroying Webrtc Client");
}

void WebrtcClient::init(boost::shared_ptr<WebrtcClient>& keep_alive_ptr)
{
  keep_alive_this_ = keep_alive_ptr;
}
void WebrtcClient::invalidate()
{
  keep_alive_this_.reset();
}
bool WebrtcClient::valid()
{
  return keep_alive_this_ != nullptr;
}


bool WebrtcClient::initPeerConnection()
{
  if(!valid()) {
    ROS_ERROR("Tried to initialize invalidated webrtc client");
    return false;
  }
  if (!peer_connection_)
  {
    webrtc::PeerConnectionInterface::IceServers servers;
    WebrtcClientWeakPtr weak_this(keep_alive_this_);
    webrtc_observer_proxy_ = new rtc::RefCountedObject<WebrtcClientObserverProxy>(weak_this);
    peer_connection_ = peer_connection_factory_->CreatePeerConnection(
            webrtc::PeerConnectionInterface::RTCConfiguration(),
            nullptr,
            nullptr,
            webrtc_observer_proxy_.get()
    );
    if (!peer_connection_.get())
    {
      ROS_WARN("Could not create peer connection");
      invalidate();
      return false;
    }
    return true;
  }
  else
  {
    return true;
  }
}

class MessageHandlerImpl : public MessageHandler {
public:
  MessageHandlerImpl(WebrtcClientWeakPtr weak_this) : weak_this_(weak_this){}
  void handle_message(MessageHandler::Type type, const std::string& raw)
  {
    WebrtcClientPtr _this = weak_this_.lock();
    if (_this)
      _this->signaling_thread_->Invoke<void>(RTC_FROM_HERE, rtc::Bind(&WebrtcClient::handle_message,
						       _this.get(), type, raw));
  }
private:
  WebrtcClientWeakPtr weak_this_;
};

MessageHandler* WebrtcClient::createMessageHandler()
{
  return new MessageHandlerImpl(keep_alive_this_);
}

void WebrtcClient::ping_timer_callback(const ros::TimerEvent& event)
{
  try
  {
    signaling_channel_->sendPingMessage();
  }
  catch (...)
  {
    //signaling channel probably broken
    if (valid())
    {
      ROS_WARN("Connection broken");
      invalidate();
    }
  }
}


class DummySetSessionDescriptionObserver
  : public webrtc::SetSessionDescriptionObserver
{
public:
  static DummySetSessionDescriptionObserver* Create()
  {
    return
      new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
  }
  virtual void OnSuccess()
  {
    //ROS_DEBUG(__FUNCTION__);
  }
  virtual void OnFailure(const std::string& error)
  {
    //ROS_WARN_STREAM(__FUNCTION__ << " " << error);
  }

protected:
  DummySetSessionDescriptionObserver() {}
  ~DummySetSessionDescriptionObserver() {}
};

static bool parseUri(const std::string& uri, std::string* scheme_name, std::string* path) {
  size_t split = uri.find_first_of(':');
  if(split == std::string::npos)
    return false;
  *scheme_name = uri.substr(0, split);
  if(uri.length() > split + 1)
    *path = uri.substr(split + 1, uri.length() - split - 1);
  else
    *path = "";
  return true;
}

void WebrtcClient::handle_message(MessageHandler::Type type, const std::string& raw)
{
  if (type == MessageHandler::TEXT)
  {
    Json::Reader reader;
    Json::Value message_json;
    if (!reader.parse(raw, message_json))
    {
      ROS_WARN_STREAM("Could not parse message: " << raw);
      invalidate();
      return;
    }

    if (ConfigureMessage::isConfigure(message_json))
    {
      ConfigureMessage message;
      if (!message.fromJson(message_json))
      {
        ROS_WARN("Can't parse received configure message.");
        invalidate();
        return;
      }

      if (!initPeerConnection())
      {
        ROS_WARN("Failed to initialize peer connection");
        return;
      }

      ROS_DEBUG("Configuring webrtc connection");

      for(const ConfigureAction& action: message.actions)
      {
	// Macro that simply checks if a key is specified and will ignore the action
	// is not specified
#define FIND_PROPERTY_OR_CONTINUE(key, name)				\
	if(action.properties.find(key) == action.properties.end()) {	\
	  ROS_WARN_STREAM("No " << #name << " specified");		\
	  continue;							\
	}								\
	std::string name = action.properties.at(key)
	// END OF MACRO

	if(action.type == ConfigureAction::kAddStreamActionName) {
	  FIND_PROPERTY_OR_CONTINUE("id", stream_id);

          rtc::scoped_refptr<webrtc::MediaStreamInterface> stream = peer_connection_factory_->CreateLocalMediaStream(stream_id);

          if (!peer_connection_->AddStream(stream))
          {
            ROS_WARN("Adding stream to PeerConnection failed");
	    continue;
          }
	}
	else if(action.type == ConfigureAction::kRemoveStreamActionName) {
	  FIND_PROPERTY_OR_CONTINUE("id", stream_id);

          rtc::scoped_refptr<webrtc::MediaStreamInterface> stream = peer_connection_factory_->CreateLocalMediaStream(stream_id);

	  if(!stream) {
	    ROS_WARN_STREAM("Stream not found with id: " << stream_id);
	    continue;
	  }
          peer_connection_->RemoveStream(stream);
	}
	else if(action.type == ConfigureAction::kAddVideoTrackActionName) {
	  FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
	  FIND_PROPERTY_OR_CONTINUE("id", track_id);
	  FIND_PROPERTY_OR_CONTINUE("src", src);

	  std::string video_type;
	  std::string video_path;
	  if(!parseUri(src, &video_type, &video_path)) {
	    ROS_WARN_STREAM("Invalid URI: " << src);
	    continue;
	  }

          webrtc::MediaStreamInterface* stream = peer_connection_->local_streams()->find(stream_id);
	  if(!stream) {
	    ROS_WARN_STREAM("Stream not found with id: " << stream_id);
	    continue;
	  }

	  if(video_type == "ros_image") {
            ROS_DEBUG_STREAM("Subscribing to ROS topic: " << video_path);
            cricket::VideoCapturer* capturer = new RosVideoCapturer(itf_, video_path, transport_);

            rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
              peer_connection_factory_->CreateVideoTrack(
                track_id,
                peer_connection_factory_->CreateVideoSource(capturer, NULL)));
            stream->AddTrack(video_track);
	  }
	  else {
	    ROS_WARN_STREAM("Unknwon video source type: " << video_type);
	  }

	}
	else if(action.type == ConfigureAction::kAddAudioTrackActionName) {
	  FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
	  FIND_PROPERTY_OR_CONTINUE("id", track_id);
	  FIND_PROPERTY_OR_CONTINUE("src", src);

	  std::string audio_type;
	  std::string audio_path;
	  if(!parseUri(src, &audio_type, &audio_path)) {
	    ROS_WARN_STREAM("Invalid URI: " << src);
	    continue;
	  }

          webrtc::MediaStreamInterface* stream = peer_connection_->local_streams()->find(stream_id);
	  if(!stream) {
	    ROS_WARN_STREAM("Stream not found with id: " << stream_id);
	    continue;
	  }

	  if(audio_type == "local") {
	    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
	      peer_connection_factory_->CreateAudioTrack(
	        track_id,
		peer_connection_factory_->CreateAudioSource(NULL)));
            stream->AddTrack(audio_track);
	  }
	  else {
	    ROS_WARN_STREAM("Unknwon video source type: " << audio_type);
	  }

	}
	else if(action.type == ConfigureAction::kExpectStreamActionName) {
	  FIND_PROPERTY_OR_CONTINUE("id", stream_id);
	  if(expected_streams_.find(stream_id) != expected_streams_.end()) {
	    ROS_WARN_STREAM("Stream id: " << stream_id << " is already expected");
	    continue;
	  }
	  expected_streams_[stream_id] = std::map<std::string, std::string>();
	}
	else if(action.type == ConfigureAction::kExpectVideoTrackActionName) {
	  FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
	  FIND_PROPERTY_OR_CONTINUE("id", track_id);
	  FIND_PROPERTY_OR_CONTINUE("dest", dest);

	  if(expected_streams_.find(stream_id) == expected_streams_.end()) {
	    ROS_WARN_STREAM("Stream id: " << stream_id << " is not expected");
	    continue;
	  }
	  if(expected_streams_[stream_id].find(track_id) != expected_streams_[stream_id].end()) {
	    ROS_WARN_STREAM("Track id: " << track_id << " is already expected in stream id: " << stream_id);
	    continue;
	  }
	  expected_streams_[stream_id][track_id] = dest;
	}
	else {
	  ROS_WARN_STREAM("Unknown configure action type: " << action.type);
	}
      }

      peer_connection_->CreateOffer(webrtc_observer_proxy_.get(), &media_constraints_);
    }
    else if (SdpMessage::isSdpAnswer(message_json))
    {
      SdpMessage message;

      if (!message.fromJson(message_json))
      {
        ROS_WARN("Can't parse received session description message.");
        invalidate();
        return;
      }

      webrtc::SessionDescriptionInterface* session_description(message.createSessionDescription());
      if (!session_description)
      {
        ROS_WARN("Can't create session description");
        invalidate();
        return;
      }

      ROS_DEBUG_STREAM("Received remote description: " << message.sdp);
      peer_connection_->SetRemoteDescription(DummySetSessionDescriptionObserver::Create(), session_description);
    }
    else if (IceCandidateMessage::isIceCandidate(message_json))
    {
      IceCandidateMessage message;
      if (!message.fromJson(message_json))
      {
        ROS_WARN("Can't parse received ice candidate message.");
        invalidate();
        return;
      }

      std::unique_ptr<webrtc::IceCandidateInterface> candidate(message.createIceCandidate());
      if (!candidate.get())
      {
        ROS_WARN("Can't parse received candidate message.");
        invalidate();
        return;
      }
      if (!peer_connection_->AddIceCandidate(candidate.get()))
      {
        ROS_WARN("Failed to apply the received candidate");
        invalidate();
        return;
      }
      ROS_DEBUG_STREAM("Received remote candidate :" << message.toJson());
      return;
    }
    else
    {
      std::string message_type;
      WebrtcRosMessage::getType(message_json, &message_type);
      ROS_WARN_STREAM("Unexpected message type: " << message_type << ": " << raw);
    }
  }
  else if (type == MessageHandler::PONG)
  {
    // got a pong from the last ping
  }
  else if (type == MessageHandler::CLOSE)
  {
    invalidate();
  }
  else
  {
    ROS_WARN_STREAM("Unexpected signaling message type: " << type << ": " << raw);
  }
}

void WebrtcClient::OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface* description)
{
  peer_connection_->SetLocalDescription(DummySetSessionDescriptionObserver::Create(), description);

  SdpMessage message;
  if (message.fromSessionDescription(*description))
  {
    ROS_DEBUG_STREAM("Created local description: " << message.sdp);
    signaling_channel_->sendTextMessage(message.toJson());
  }
  else
  {
    ROS_WARN("Failed to serialize description");
  }
}
void WebrtcClient::OnSessionDescriptionFailure(const std::string& error)
{
  ROS_WARN_STREAM("Could not create local description: " << error);
  invalidate();
}
void WebrtcClient::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
  IceCandidateMessage message;
  if (message.fromIceCandidate(*candidate))
  {
    ROS_DEBUG_STREAM("Got local ICE candidate: " << message.toJson());
    signaling_channel_->sendTextMessage(message.toJson());
  }
  else
  {
    ROS_WARN("Failed to serialize local candidate");
  }
}

void WebrtcClient::OnAddRemoteStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  std::string stream_id = media_stream->label();
  if(expected_streams_.find(stream_id) != expected_streams_.end()) {
    for(auto& track : media_stream->GetVideoTracks()) {
      if(expected_streams_[stream_id].find(track->id()) != expected_streams_[stream_id].end()) {
	std::string dest = expected_streams_[stream_id][track->id()];

	std::string video_type;
	std::string video_path;
	if(!parseUri(dest, &video_type, &video_path)) {
	  ROS_WARN_STREAM("Invalid URI: " << dest);
	  continue;
	}

	if(video_type == "ros_image") {
	  boost::shared_ptr<RosVideoRenderer> renderer(new RosVideoRenderer(it_, video_path));
	  track->AddOrUpdateSink(renderer.get(), rtc::VideoSinkWants());
	  video_renderers_[stream_id].push_back(renderer);
	}
	else {
	  ROS_WARN_STREAM("Unknown video destination type: " << video_type);
	}

      }
      else {
	ROS_WARN_STREAM("Unexpected video track: " << track->id());
      }
    }
    // Currently audio tracks play to system default output without any action taken
    // It does not appear to be simple to change this
  }
  else {
    ROS_WARN_STREAM("Unexpected stream: " << stream_id);
  }
}
void WebrtcClient::OnRemoveRemoteStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  std::string stream_id = media_stream->label();
  video_renderers_.erase(stream_id);
}

}
