#include <ros/ros.h>
#include <webrtc_ros/webrtc_client.h>
#include "webrtc_ros/webrtc_ros_message.h"
#include "webrtc_ros/sdp_message.h"
#include "webrtc_ros/ice_candidate_message.h"
#include "webrtc/base/json.h"
#include "talk/media/devices/devicemanager.h"
#include "talk/app/webrtc/videosourceinterface.h"

namespace webrtc_ros {

WebrtcClientObserverProxy::WebrtcClientObserverProxy(boost::weak_ptr<WebrtcClient> client_weak)
  : client_weak_(client_weak) {}

void WebrtcClientObserverProxy::OnSuccess(webrtc::SessionDescriptionInterface* description) {
  boost::shared_ptr<WebrtcClient> client = client_weak_.lock();
  if(client)
    client->OnSessionDescriptionSuccess(description);
}
void WebrtcClientObserverProxy::OnFailure(const std::string& error) {
  boost::shared_ptr<WebrtcClient> client = client_weak_.lock();
  if(client)
    client->OnSessionDescriptionFailure(error);
}
void WebrtcClientObserverProxy::OnAddStream(webrtc::MediaStreamInterface* media_stream){
  boost::shared_ptr<WebrtcClient> client = client_weak_.lock();
  if(client)
    client->OnAddRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnRemoveStream(webrtc::MediaStreamInterface*){
}
void WebrtcClientObserverProxy::OnDataChannel(webrtc::DataChannelInterface*){
}
void WebrtcClientObserverProxy::OnRenegotiationNeeded(){
}
void WebrtcClientObserverProxy::OnIceCandidate(const webrtc::IceCandidateInterface* candidate){
  boost::shared_ptr<WebrtcClient> client = client_weak_.lock();
  if(client)
    client->OnIceCandidate(candidate);
}


WebrtcClient::WebrtcClient(ros::NodeHandle& nh, cpp_web_server::WebsocketConnectionPtr signaling_channel)
  : nh_(nh), it_(nh_), signaling_channel_(signaling_channel),
    ros_media_device_manager_(it_){
  peer_connection_factory_  = webrtc::CreatePeerConnectionFactory();
  if (!peer_connection_factory_.get()) {
    ROS_WARN("Could not create peer connection factory");
    invalidate();
    return;
  }
  peer_connection_constraints_.SetAllowDtlsSctpDataChannels();
  media_constraints_.SetMandatoryReceiveVideo(true);
  media_constraints_.SetMandatoryReceiveAudio(true);
  ping_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&WebrtcClient::ping_timer_callback, this, _1));
}
WebrtcClient::~WebrtcClient(){
  ROS_INFO("Webrtc Client Destroyed");
}

void WebrtcClient::init() {
  keep_alive_this_ = shared_from_this();
}
void WebrtcClient::invalidate() {
  keep_alive_this_.reset();
}
bool WebrtcClient::valid() {
  return keep_alive_this_;
}


bool WebrtcClient::initPeerConnection() {
  if(!peer_connection_) {
    webrtc::PeerConnectionInterface::IceServers servers;
    webrtc_observer_proxy_ = new rtc::RefCountedObject<WebrtcClientObserverProxy>(boost::weak_ptr<WebrtcClient>(shared_from_this()));
    peer_connection_ = peer_connection_factory_->CreatePeerConnection(servers,
								      &peer_connection_constraints_,
								      NULL,
								      NULL,
								      webrtc_observer_proxy_.get());
    if (!peer_connection_.get()) {
      ROS_WARN("Could not create peer connection");
      invalidate();
      return false;
    }
    return true;
  }
  else {
    return true;
  }
}


cpp_web_server::WebsocketConnection::MessageHandler WebrtcClient::createMessageHandler() {
  return boost::bind(&WebrtcClient::static_handle_message, boost::weak_ptr<WebrtcClient>(shared_from_this()), _1);
}

void WebrtcClient::ping_timer_callback(const ros::TimerEvent& event) {
  try {
    signaling_channel_->sendPingMessage();
  } catch(...) {
    //signaling channel probably broken
    if(valid()) {
      ROS_WARN("Connection broken");
      invalidate();
    }
  }
}


class DummySetSessionDescriptionObserver
    : public webrtc::SetSessionDescriptionObserver {
 public:
  static DummySetSessionDescriptionObserver* Create() {
    return
        new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
  }
  virtual void OnSuccess() {
    ROS_DEBUG(__FUNCTION__);
  }
  virtual void OnFailure(const std::string& error) {
    ROS_WARN_STREAM(__FUNCTION__ << " " << error);
  }

 protected:
  DummySetSessionDescriptionObserver() {}
  ~DummySetSessionDescriptionObserver() {}
};

void WebrtcClient::static_handle_message(boost::weak_ptr<WebrtcClient> weak_this,
					 const cpp_web_server::WebsocketMessage& message) {
  boost::shared_ptr<WebrtcClient> _this = weak_this.lock();
  if(_this)
    _this->handle_message(message);
}


void WebrtcClient::handle_message(const cpp_web_server::WebsocketMessage& message) {
  if(message.type == cpp_web_server::WebsocketMessage::type_text) {
    Json::Reader reader;
    Json::Value message_json;
    if (!reader.parse(message.content, message_json)) {
      ROS_WARN_STREAM("Could not parse message: " << message.content);
      invalidate();
      return;
    }

    if(ConfigureMessage::isConfigure(message_json)){
      ConfigureMessage message;
      if(!message.fromJson(message_json)) {
	ROS_WARN("Can't parse received configure message.");
	invalidate();
	return;
      }

      last_configuration_ = message;

      if(!initPeerConnection()) {
	ROS_WARN("Failed to initialize peer connection");
	return;
      }

      ROS_DEBUG("Configuring webrtc connection");

      rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
	peer_connection_factory_->CreateLocalMediaStream("ros_media_stream");

      if(!message.subscribed_video_topic.empty()) {
	ROS_DEBUG_STREAM("Subscribing to ROS topic: " << message.subscribed_video_topic);
	cricket::Device device(message.subscribed_video_topic, message.subscribed_video_topic);
	cricket::VideoCapturer* capturer = ros_media_device_manager_.CreateVideoCapturer(device);

	rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
            peer_connection_factory_->CreateVideoTrack(
                message.subscribed_video_topic,
                peer_connection_factory_->CreateVideoSource(capturer,
                                                            NULL)));
	stream->AddTrack(video_track);
      }

      if (!peer_connection_->AddStream(stream)) {
        ROS_WARN("Adding stream to PeerConnection failed");
	invalidate();
      }

      peer_connection_->CreateOffer(webrtc_observer_proxy_.get(), &media_constraints_);
    }
    else if(SdpMessage::isSdpAnswer(message_json)){
      SdpMessage message;

      if(!message.fromJson(message_json)) {
	ROS_WARN("Can't parse received session description message.");
	invalidate();
	return;
      }

      webrtc::SessionDescriptionInterface* session_description(message.createSessionDescription());
      if (!session_description) {
	ROS_WARN("Can't create session description");
	invalidate();
	return;
      }

      ROS_DEBUG_STREAM("Received remote description: " << message.sdp);
      peer_connection_->SetRemoteDescription(DummySetSessionDescriptionObserver::Create(), session_description);
    } else if(IceCandidateMessage::isIceCandidate(message_json)){
      IceCandidateMessage message;
      if(!message.fromJson(message_json)) {
	ROS_WARN("Can't parse received ice candidate message.");
	invalidate();
	return;
      }

      rtc::scoped_ptr<webrtc::IceCandidateInterface> candidate(message.createIceCandidate());
      if (!candidate.get()) {
	ROS_WARN("Can't parse received candidate message.");
	invalidate();
	return;
      }
      if (!peer_connection_->AddIceCandidate(candidate.get())) {
	ROS_WARN("Failed to apply the received candidate");
	invalidate();
	return;
      }
      ROS_DEBUG_STREAM("Received remote candidate :" << message.toJson());
      return;
    }
    else {
      std::string type;
      WebrtcRosMessage::getType(message_json, &type);
      ROS_WARN_STREAM("Unexpected message type: " << type << ": " << message.content);
    }
  }
  else if(message.type == cpp_web_server::WebsocketMessage::type_pong) {
    // got a pong from the last ping
  }
  else{
    ROS_WARN_STREAM("Unexpected websocket message type: " << message.type << ": " << message.content);
  }
}

void WebrtcClient::OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface* description) {
  peer_connection_->SetLocalDescription(DummySetSessionDescriptionObserver::Create(), description);

  SdpMessage message;
  if(message.fromSessionDescription(*description)) {
    ROS_DEBUG_STREAM("Created local description: " << message.sdp);
    signaling_channel_->sendTextMessage(message.toJson());
  }
  else {
    ROS_WARN("Failed to serialize description");
  }
}
void WebrtcClient::OnSessionDescriptionFailure(const std::string& error) {
  ROS_WARN_STREAM("Could not create local description: " << error);
  invalidate();
}
void WebrtcClient::OnIceCandidate(const webrtc::IceCandidateInterface* candidate){
  IceCandidateMessage message;
  if(message.fromIceCandidate(*candidate)) {
    ROS_DEBUG_STREAM("Got local ICE candidate: " << message.toJson());
    signaling_channel_->sendTextMessage(message.toJson());
  }
  else {
    ROS_WARN("Failed to serialize local candidate");
  }
}

void WebrtcClient::OnAddRemoteStream(webrtc::MediaStreamInterface* media_stream) {
  webrtc::VideoTrackVector video_tracks = media_stream->GetVideoTracks();
  webrtc::AudioTrackVector audio_tracks = media_stream->GetAudioTracks();
  ROS_DEBUG("Got remote stream video: %ld, audio: %ld", video_tracks.size(), audio_tracks.size());
  if(video_tracks.size() > 0 && !last_configuration_.published_video_topic.empty()) {
    ROS_DEBUG_STREAM("Got remote video track, kind=" << video_tracks[0]->kind() << ", id=" << video_tracks[0]->id());
    cricket::Device device(last_configuration_.published_video_topic, last_configuration_.published_video_topic);
    video_renderer_ = boost::shared_ptr<RosVideoRenderer>(ros_media_device_manager_.CreateVideoRenderer(device));
    video_tracks[0]->AddRenderer(video_renderer_.get());
  }
}





}
