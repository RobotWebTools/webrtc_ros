#ifndef WEBRTC_ROS_WEBRTC_CLIENT_H_
#define WEBRTC_ROS_WEBRTC_CLIENT_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "webrtc_ros/ros_video_renderer.h"
#include "talk/app/webrtc/mediastreaminterface.h"
#include "talk/app/webrtc/peerconnectioninterface.h"
#include "talk/app/webrtc/test/fakeconstraints.h"
#include "webrtc_ros/configure_message.h"
#include "webrtc_ros/webrtc_web_server.h"
#include "webrtc/base/thread.h"

namespace webrtc_ros
{

class WebrtcClient;
typedef boost::shared_ptr<WebrtcClient> WebrtcClientPtr;
typedef boost::weak_ptr<WebrtcClient> WebrtcClientWeakPtr;

class WebrtcClientObserverProxy : public webrtc::PeerConnectionObserver,
  public webrtc::CreateSessionDescriptionObserver
{
public:
  WebrtcClientObserverProxy(WebrtcClientWeakPtr client_weak);

  void OnSuccess(webrtc::SessionDescriptionInterface*);
  void OnFailure(const std::string&);
  void OnAddStream(webrtc::MediaStreamInterface*);
  void OnRemoveStream(webrtc::MediaStreamInterface*);
  void OnDataChannel(webrtc::DataChannelInterface*);
  void OnRenegotiationNeeded();
  void OnIceCandidate(const webrtc::IceCandidateInterface*);

private:
  WebrtcClientWeakPtr client_weak_;

};

class MessageHandlerImpl;
class WebrtcClient : private boost::noncopyable
{
public:
  WebrtcClient(ros::NodeHandle& nh, SignalingChannel *signaling_channel);
  ~WebrtcClient();
  MessageHandler* createMessageHandler();

  void init(boost::shared_ptr<WebrtcClient>& keep_alive_ptr);
  void invalidate();
  bool valid();


private:
  WebrtcClientPtr keep_alive_this_;

  bool initPeerConnection();

  void ping_timer_callback(const ros::TimerEvent&);

  void handle_message(MessageHandler::Type type, const std::string& message);

  void OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface*);
  void OnSessionDescriptionFailure(const std::string&);
  void OnAddRemoteStream(webrtc::MediaStreamInterface*);
  void OnRemoveRemoteStream(webrtc::MediaStreamInterface*);
  void OnIceCandidate(const webrtc::IceCandidateInterface*);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  boost::scoped_ptr<SignalingChannel> signaling_channel_;

  rtc::Thread *signaling_thread_;

  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> peer_connection_factory_;
  std::map<std::string, std::vector<boost::shared_ptr<RosVideoRenderer>>> video_renderers_;
  rtc::scoped_refptr<WebrtcClientObserverProxy> webrtc_observer_proxy_;
  webrtc::FakeConstraints peer_connection_constraints_;
  webrtc::FakeConstraints media_constraints_;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection_;

  std::map<std::string, std::map<std::string, std::string>> expected_streams_;

  ros::Timer ping_timer_;

  friend WebrtcClientObserverProxy;
  friend MessageHandlerImpl;
};

}

#endif
