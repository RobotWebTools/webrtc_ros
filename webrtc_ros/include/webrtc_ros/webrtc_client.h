#ifndef WEBRTC_ROS_WEBRTC_CLIENT_H_
#define WEBRTC_ROS_WEBRTC_CLIENT_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "webrtc_ros/ros_media_device_manager.h"
#include <cpp_web_server/websocket_request_handler.hpp>
#include "talk/app/webrtc/mediastreaminterface.h"
#include "talk/app/webrtc/peerconnectioninterface.h"
#include "talk/app/webrtc/test/fakeconstraints.h"
#include "webrtc_ros/configure_message.h"

namespace webrtc_ros {

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


class WebrtcClient : public boost::enable_shared_from_this<WebrtcClient>,
  private boost::noncopyable
 {
 public:
  WebrtcClient(ros::NodeHandle& nh, cpp_web_server::WebsocketConnectionPtr signaling_channel);
  ~WebrtcClient();
  cpp_web_server::WebsocketConnection::MessageHandler createMessageHandler();

  void init();
  void invalidate();
  bool valid();


 private:
  WebrtcClientPtr keep_alive_this_;

  bool initPeerConnection();

  void ping_timer_callback(const ros::TimerEvent&);

  static void static_handle_message(WebrtcClientWeakPtr weak_this,
				    const cpp_web_server::WebsocketMessage& message);
  void handle_message(const cpp_web_server::WebsocketMessage& message);

  void OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface*);
  void OnSessionDescriptionFailure(const std::string&);
  void OnAddRemoteStream(webrtc::MediaStreamInterface*);
  void OnIceCandidate(const webrtc::IceCandidateInterface*);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  cpp_web_server::WebsocketConnectionPtr signaling_channel_;

  RosMediaDeviceManager ros_media_device_manager_;
  ConfigureMessage last_configuration_;

  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> peer_connection_factory_;
  boost::shared_ptr<RosVideoRenderer> video_renderer_;
  rtc::scoped_refptr<WebrtcClientObserverProxy> webrtc_observer_proxy_;
  webrtc::FakeConstraints peer_connection_constraints_;
  webrtc::FakeConstraints media_constraints_;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection_;
  rtc::scoped_refptr<webrtc::MediaStreamInterface> stream_;

  ros::Timer ping_timer_;

  friend WebrtcClientObserverProxy;
};

}

#endif
