#include <ros/ros.h>
#include <webrtc_ros/webrtc_client.h>
#include "webrtc_ros/ros_video_capturer.h"
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


WebrtcClient::WebrtcClient(cpp_web_server::WebsocketConnectionPtr signaling_channel)
  : signaling_channel_(signaling_channel) {

  peer_connection_factory_  = webrtc::CreatePeerConnectionFactory();
  if (!peer_connection_factory_.get()) {
    ROS_ERROR("Could not create peer connection factory");
  }
  constraints_.SetAllowDtlsSctpDataChannels();
}


cpp_web_server::WebsocketConnection::MessageHandler WebrtcClient::createMessageHandler() {
  return boost::bind(&WebrtcClient::static_handle_message, boost::weak_ptr<WebrtcClient>(shared_from_this()), _1);
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
    ROS_INFO(__FUNCTION__);
  }
  virtual void OnFailure(const std::string& error) {
    ROS_ERROR_STREAM(__FUNCTION__ << " " << error);
  }

 protected:
  DummySetSessionDescriptionObserver() {}
  ~DummySetSessionDescriptionObserver() {}
};

static cricket::VideoCapturer* OpenVideoCaptureDevice() {
  rtc::scoped_ptr<cricket::DeviceManagerInterface> dev_manager(
      cricket::DeviceManagerFactory::Create());
  if (!dev_manager->Init()) {
    ROS_ERROR("Can't create device manager");
    return NULL;
  }
  dev_manager->SetVideoDeviceCapturerFactory(new webrtc_ros::RosVideoCapturerFactory());
  std::vector<cricket::Device> devs;
  if (!dev_manager->GetVideoCaptureDevices(&devs)) {
    ROS_ERROR("Can't enumerate video devices");
    return NULL;
  }
  std::vector<cricket::Device>::iterator dev_it = devs.begin();
  cricket::VideoCapturer* capturer = NULL;
  for (; dev_it != devs.end(); ++dev_it) {
    capturer = dev_manager->CreateVideoCapturer(*dev_it);
    if (capturer != NULL)
      break;
  }
  return capturer;
}


void WebrtcClient::handle_message(const cpp_web_server::WebsocketMessage& message){
  if(message.type == cpp_web_server::WebsocketMessage::type_text) {
    std::cout << "Message: " << message.content << std::endl;

    Json::Reader reader;
    Json::Value jmessage;
    if (!reader.parse(message.content, jmessage)) {
      ROS_WARN_STREAM("Received unknown message. " << message.content);
      return;
    }
    std::string type;
    std::string json_object;
    GetStringFromJsonObject(jmessage, "type", &type);
    if (!type.empty()) {
      webrtc::PeerConnectionInterface::IceServers servers;

      if(!webrtc_observer_proxy_.get())
	webrtc_observer_proxy_ = new rtc::RefCountedObject<WebrtcClientObserverProxy>(boost::weak_ptr<WebrtcClient>(shared_from_this()));
      peer_connection_ = peer_connection_factory_->CreatePeerConnection(servers,
									&constraints_,
									NULL,
									NULL,
									webrtc_observer_proxy_.get());
      if (!peer_connection_.get()) {
	ROS_ERROR("Could not create peer connection");
	return;
      }
      std::string sdp;
      if (!GetStringFromJsonObject(jmessage, "sdp", &sdp)) {
	ROS_WARN("Can't parse received session description message.");
	return;
      }
      webrtc::SessionDescriptionInterface* session_description(
							       webrtc::CreateSessionDescription(type, sdp));
      if (!session_description) {
	ROS_WARN("Can't parse received session description message.");
	return;
      }
      ROS_INFO_STREAM(" Received session description :" << message.content);
      peer_connection_->SetRemoteDescription(
					     DummySetSessionDescriptionObserver::Create(), session_description);

      rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
          peer_connection_factory_->CreateAudioTrack(
              "audio_label", peer_connection_factory_->CreateAudioSource(NULL)));

      cricket::VideoCapturer* capturer = OpenVideoCaptureDevice();
      rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
          peer_connection_factory_->CreateVideoTrack(
              "video_label",
              peer_connection_factory_->CreateVideoSource(capturer,
                                                          NULL)));
      rtc::scoped_refptr<webrtc::MediaStreamInterface> stream =
          peer_connection_factory_->CreateLocalMediaStream("stream_label");

      stream->AddTrack(audio_track);
      stream->AddTrack(video_track);

      if (!peer_connection_->AddStream(stream)) {
        ROS_ERROR("Adding stream to PeerConnection failed");
      }

      if (session_description->type() ==
	  webrtc::SessionDescriptionInterface::kOffer) {
	peer_connection_->CreateAnswer(webrtc_observer_proxy_.get(), NULL);
      }
    } else {
      std::string sdp_mid;
      int sdp_mlineindex = 0;
      std::string sdp;
      if (!GetStringFromJsonObject(jmessage, "sdpMid", &sdp_mid) ||
	  !GetIntFromJsonObject(jmessage, "sdpMLineIndex",
				&sdp_mlineindex) ||
	  !GetStringFromJsonObject(jmessage, "candidate", &sdp)) {
	ROS_WARN("Can't parse received message.");
	return;
      }
      rtc::scoped_ptr<webrtc::IceCandidateInterface> candidate(
							       webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp));
      if (!candidate.get()) {
	ROS_WARN("Can't parse received candidate message.");
	return;
      }
      if (!peer_connection_->AddIceCandidate(candidate.get())) {
	ROS_WARN("Failed to apply the received candidate");
	return;
      }
      ROS_INFO_STREAM(" Received candidate :" << message.content);
      return;
    }
  }
  else{
    std::cout << "Non-text Message: " << message.type << ": " << message.content << std::endl;
  }
}

void WebrtcClient::OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface* description) {
  std::cout << "Got description" <<  std::endl;
  peer_connection_->SetLocalDescription(DummySetSessionDescriptionObserver::Create(), description);
  Json::StyledWriter writer;
  Json::Value jmessage;
  jmessage["type"] = description->type();
  std::string sdp;
  description->ToString(&sdp);
  jmessage["sdp"] = sdp;

  cpp_web_server::WebsocketMessage m;
  m.type = cpp_web_server::WebsocketMessage::type_text;
  m.content = writer.write(jmessage);
  signaling_channel_->sendMessage(m);
}
void WebrtcClient::OnSessionDescriptionFailure(const std::string& error) {
  std::cout << "Description failure: " << error <<  std::endl;
}
void WebrtcClient::OnIceCandidate(const webrtc::IceCandidateInterface* candidate){
  ROS_INFO_STREAM(__FUNCTION__ << " " << candidate->sdp_mline_index());
  Json::StyledWriter writer;
  Json::Value jmessage;

  jmessage["sdpMid"] = candidate->sdp_mid();
  jmessage["sdpMLineIndex"] = candidate->sdp_mline_index();
  std::string sdp;
  if (!candidate->ToString(&sdp)) {
    ROS_ERROR("Failed to serialize candidate");
    return;
  }
  jmessage["candidate"] = sdp;

  cpp_web_server::WebsocketMessage m;
  m.type = cpp_web_server::WebsocketMessage::type_text;
  m.content = writer.write(jmessage);
  signaling_channel_->sendMessage(m);
}




}
