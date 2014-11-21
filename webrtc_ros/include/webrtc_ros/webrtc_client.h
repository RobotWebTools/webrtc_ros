#ifndef WEBRTC_ROS_WEBRTC_CLIENT_H_
#define WEBRTC_ROS_WEBRTC_CLIENT_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <cpp_web_server/websocket_request_handler.hpp>
#include "talk/app/webrtc/peerconnectioninterface.h"
#include "talk/app/webrtc/test/fakeconstraints.h"

namespace webrtc_ros {

class WebrtcClient;

class WebrtcClientObserverProxy : public webrtc::PeerConnectionObserver,
  public webrtc::CreateSessionDescriptionObserver
{
 public:
  WebrtcClientObserverProxy(boost::weak_ptr<WebrtcClient> client_weak);

  void OnSuccess(webrtc::SessionDescriptionInterface*);
  void OnFailure(const std::string&);
  void OnAddStream(webrtc::MediaStreamInterface*);
  void OnRemoveStream(webrtc::MediaStreamInterface*);
  void OnDataChannel(webrtc::DataChannelInterface*);
  void OnRenegotiationNeeded();
  void OnIceCandidate(const webrtc::IceCandidateInterface*);

 private:
  boost::weak_ptr<WebrtcClient> client_weak_;

};


class WebrtcClient : public boost::enable_shared_from_this<WebrtcClient>,
  private boost::noncopyable
 {
 public:
  WebrtcClient(ros::NodeHandle& nh, cpp_web_server::WebsocketConnectionPtr signaling_channel);
  cpp_web_server::WebsocketConnection::MessageHandler createMessageHandler();


 private:
  void ping_timer_callback(const ros::TimerEvent&);

  static void static_handle_message(boost::weak_ptr<WebrtcClient> weak_this,
				    const cpp_web_server::WebsocketMessage& message);
  void handle_message(const cpp_web_server::WebsocketMessage& message);

  void OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface*);
  void OnSessionDescriptionFailure(const std::string&);
  void OnIceCandidate(const webrtc::IceCandidateInterface*);

  bool is_broken_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  cpp_web_server::WebsocketConnectionPtr signaling_channel_;

  rtc::scoped_refptr<WebrtcClientObserverProxy> webrtc_observer_proxy_;
  webrtc::FakeConstraints constraints_;
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> peer_connection_factory_;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection_;

  ros::Timer ping_timer_;

  friend WebrtcClientObserverProxy;
};

}

#endif
