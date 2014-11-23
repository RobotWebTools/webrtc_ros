#include <ros/ros.h>
#include <webrtc_ros/webrtc_client.h>
#include "webrtc_ros/ros_media_device_manager.h"
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
void WebrtcClientObserverProxy::OnAddStream(webrtc::MediaStreamInterface*){
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
  : is_broken_(false), nh_(nh), it_(nh_), signaling_channel_(signaling_channel) {
  peer_connection_factory_  = webrtc::CreatePeerConnectionFactory();
  if (!peer_connection_factory_.get()) {
    ROS_WARN("Could not create peer connection factory");
    is_broken_ = true;
  }
  else {
    constraints_.SetAllowDtlsSctpDataChannels();
    ping_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&WebrtcClient::ping_timer_callback, this, _1));
  }
}


cpp_web_server::WebsocketConnection::MessageHandler WebrtcClient::createMessageHandler() {
  return boost::bind(&WebrtcClient::static_handle_message, boost::weak_ptr<WebrtcClient>(shared_from_this()), _1);
}

void WebrtcClient::ping_timer_callback(const ros::TimerEvent& event) {
  try {
    cpp_web_server::WebsocketMessage m;
    m.type = cpp_web_server::WebsocketMessage::type_ping;
    signaling_channel_->sendMessage(m);
  } catch(...) {
    //signaling channel probably broken
    if(!is_broken_) {
      ROS_WARN("Connection broken");
      is_broken_ = true;
    }
  }
}


void WebrtcClient::static_handle_message(boost::weak_ptr<WebrtcClient> weak_this,
					 const cpp_web_server::WebsocketMessage& message){
  boost::shared_ptr<WebrtcClient> _this = weak_this.lock();
  if(_this)
    _this->handle_message(message);
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


void WebrtcClient::handle_message(const cpp_web_server::WebsocketMessage& message){
  if(message.type == cpp_web_server::WebsocketMessage::type_text) {
    Json::Reader reader;
    Json::Value message_json;
    if (!reader.parse(message.content, message_json)) {
      ROS_WARN_STREAM("Could not parse message: " << message.content);
      is_broken_ = true;
      return;
    }

    if(SdpMessage::isSdpOffer(message_json)){
      SdpMessage message;

      if(!message.fromJson(message_json)) {
	ROS_WARN("Can't parse received session description message.");
	is_broken_ = true;
	return;
      }

      webrtc::PeerConnectionInterface::IceServers servers;
      if(!webrtc_observer_proxy_.get())
	webrtc_observer_proxy_ = new rtc::RefCountedObject<WebrtcClientObserverProxy>(boost::weak_ptr<WebrtcClient>(shared_from_this()));
      peer_connection_ = peer_connection_factory_->CreatePeerConnection(servers,
									&constraints_,
									NULL,
									NULL,
									webrtc_observer_proxy_.get());
      if (!peer_connection_.get()) {
	ROS_WARN("Could not create peer connection");
	is_broken_ = true;
	return;
      }

      webrtc::SessionDescriptionInterface* session_description(message.createSessionDescription());
      if (!session_description) {
	ROS_WARN("Can't create session description");
	is_broken_ = true;
	return;
      }

      ROS_DEBUG_STREAM("Received remote description :" << message.sdp);
      peer_connection_->SetRemoteDescription(DummySetSessionDescriptionObserver::Create(), session_description);

      rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
          peer_connection_factory_->CreateAudioTrack(
              "audio_label", peer_connection_factory_->CreateAudioSource(NULL)));

      RosMediaDeviceManager dev_manager(it_);
      cricket::Device device("image", "image");
      cricket::VideoCapturer* capturer = dev_manager.CreateVideoCapturer(device);

      rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
          peer_connection_factory_->CreateVideoTrack(
              "video_label",
              peer_connection_factory_->CreateVideoSource(capturer,
                                                          NULL)));
      rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
          peer_connection_factory_->CreateLocalMediaStream("stream_label");

      //stream->AddTrack(audio_track);
      stream->AddTrack(video_track);

      if (!peer_connection_->AddStream(stream)) {
        ROS_WARN("Adding stream to PeerConnection failed");
	is_broken_ = true;
      }

      peer_connection_->CreateAnswer(webrtc_observer_proxy_.get(), NULL);
    } else if(IceCandidateMessage::isIceCandidate(message_json)){
      IceCandidateMessage message;
      if(!message.fromJson(message_json)) {
	ROS_WARN("Can't parse received ice candidate message.");
	is_broken_ = true;
	return;
      }

      rtc::scoped_ptr<webrtc::IceCandidateInterface> candidate(message.createIceCandidate());
      if (!candidate.get()) {
	ROS_WARN("Can't parse received candidate message.");
	is_broken_ = true;
	return;
      }
      if (!peer_connection_->AddIceCandidate(candidate.get())) {
	ROS_WARN("Failed to apply the received candidate");
	is_broken_ = true;
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
  ROS_DEBUG("Created local description");
  peer_connection_->SetLocalDescription(DummySetSessionDescriptionObserver::Create(), description);

  SdpMessage message;
  if(message.fromSessionDescription(*description)) {
    cpp_web_server::WebsocketMessage m;
    m.type = cpp_web_server::WebsocketMessage::type_text;
    m.content = message.toJson();
    signaling_channel_->sendMessage(m);
  }
  else {
    ROS_WARN("Failed to serialize description");
  }
}
void WebrtcClient::OnSessionDescriptionFailure(const std::string& error) {
  ROS_WARN_STREAM("Could not create local description: " << error);
  is_broken_ = true;
}
void WebrtcClient::OnIceCandidate(const webrtc::IceCandidateInterface* candidate){
  IceCandidateMessage message;
  if(message.fromIceCandidate(*candidate)) {
    ROS_DEBUG_STREAM("Got local ICE candidate: " << message.toJson());

    cpp_web_server::WebsocketMessage m;
    m.type = cpp_web_server::WebsocketMessage::type_text;
    m.content = message.toJson();
    signaling_channel_->sendMessage(m);
  }
  else {
    ROS_WARN("Failed to serialize local candidate");
  }
}




}
