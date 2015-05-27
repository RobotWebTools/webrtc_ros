#include <ros/ros.h>
#include <webrtc_ros/webrtc_client.h>
#include "webrtc_ros/webrtc_ros_message.h"
#include "webrtc_ros/sdp_message.h"
#include "webrtc_ros/ice_candidate_message.h"
#include "webrtc/base/json.h"
#include "talk/media/devices/devicemanager.h"
#include "talk/app/webrtc/videosourceinterface.h"

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
void WebrtcClientObserverProxy::OnAddStream(webrtc::MediaStreamInterface* media_stream)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnAddRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnRemoveStream(webrtc::MediaStreamInterface*)
{
}
void WebrtcClientObserverProxy::OnDataChannel(webrtc::DataChannelInterface*)
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


WebrtcClient::WebrtcClient(ros::NodeHandle& nh, SignalingChannel* signaling_channel)
  : nh_(nh), it_(nh_), signaling_channel_(signaling_channel),
    ros_media_device_manager_(it_)
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
  media_constraints_.SetMandatoryReceiveVideo(true);
  media_constraints_.SetMandatoryReceiveAudio(true);
  ping_timer_ = nh_.createTimer(ros::Duration(10.0), boost::bind(&WebrtcClient::ping_timer_callback, this, _1));
}
WebrtcClient::~WebrtcClient()
{
  ROS_INFO("Webrtc Client Destroyed");
}

void WebrtcClient::init()
{
  keep_alive_this_ = shared_from_this();
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
  if (!peer_connection_)
  {
    webrtc::PeerConnectionInterface::IceServers servers;
    WebrtcClientWeakPtr weak_this(shared_from_this());
    webrtc_observer_proxy_ = new rtc::RefCountedObject<WebrtcClientObserverProxy>(weak_this);
    peer_connection_ = peer_connection_factory_->CreatePeerConnection(servers,
                       &peer_connection_constraints_,
                       NULL,
                       NULL,
                       webrtc_observer_proxy_.get());
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
      _this->handle_message(type, raw);
  }
private:
  WebrtcClientWeakPtr weak_this_;
};

MessageHandler* WebrtcClient::createMessageHandler()
{
  return new MessageHandlerImpl(shared_from_this());
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

      last_configuration_ = message;

      if (!initPeerConnection())
      {
        ROS_WARN("Failed to initialize peer connection");
        return;
      }

      ROS_DEBUG("Configuring webrtc connection");

      if(last_stream_)
      {
	peer_connection_->RemoveStream(last_stream_);
	last_stream_ = NULL;
      }

      // Need to generate unique stream id for the stream to change
      static int i = 0;
      std::stringstream ss;
      ss << "ros_media_stream" << i++;
      last_stream_ = peer_connection_factory_->CreateLocalMediaStream(ss.str());

      if (!message.subscribed_video_topic.empty())
      {
        ROS_DEBUG_STREAM("Subscribing to ROS topic: " << message.subscribed_video_topic);
        cricket::Device device(message.subscribed_video_topic, message.subscribed_video_topic);
        cricket::VideoCapturer* capturer = ros_media_device_manager_.CreateVideoCapturer(device);

        rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
          peer_connection_factory_->CreateVideoTrack(
            message.subscribed_video_topic,
            peer_connection_factory_->CreateVideoSource(capturer,
                NULL)));
        last_stream_->AddTrack(video_track);
      }

      if (!peer_connection_->AddStream(last_stream_))
      {
        ROS_WARN("Adding stream to PeerConnection failed");
        invalidate();
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

      rtc::scoped_ptr<webrtc::IceCandidateInterface> candidate(message.createIceCandidate());
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

void WebrtcClient::OnAddRemoteStream(webrtc::MediaStreamInterface* media_stream)
{
  webrtc::VideoTrackVector video_tracks = media_stream->GetVideoTracks();
  webrtc::AudioTrackVector audio_tracks = media_stream->GetAudioTracks();
  ROS_DEBUG("Got remote stream video: %ld, audio: %ld", video_tracks.size(), audio_tracks.size());
  if (video_tracks.size() > 0 && !last_configuration_.published_video_topic.empty())
  {
    ROS_DEBUG_STREAM("Got remote video track, kind=" << video_tracks[0]->kind() << ", id=" << video_tracks[0]->id());
    cricket::Device device(last_configuration_.published_video_topic, last_configuration_.published_video_topic);
    video_renderer_ = boost::shared_ptr<RosVideoRenderer>(ros_media_device_manager_.CreateVideoRenderer(device));
    video_tracks[0]->AddRenderer(video_renderer_.get());
  }
}





}
