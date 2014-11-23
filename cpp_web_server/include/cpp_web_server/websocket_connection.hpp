#ifndef CPP_WEB_SERVER_WEBSOCKET_CONNECTION_HPP
#define CPP_WEB_SERVER_WEBSOCKET_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include "cpp_web_server/http_connection.hpp"
#include "cpp_web_server/websocket_message.hpp"

namespace cpp_web_server
{

class WebsocketHttpRequestHandler;

class WebsocketConnection;
typedef boost::shared_ptr<WebsocketConnection> WebsocketConnectionPtr;
typedef boost::weak_ptr<WebsocketConnection> WebsocketConnectionWeakPtr;

// Represents a connection to a client
// To keep the connection alive keep a shared pointer to this object
class WebsocketConnection : public boost::enable_shared_from_this<WebsocketConnection>,
			    private boost::noncopyable
{
public:
  explicit WebsocketConnection(HttpConnectionPtr connection);

  typedef boost::function<void(const WebsocketMessage& message)> MessageHandler;

  bool sendTextMessage(const std::string& content);
  bool sendPingMessage(const std::string& content = "");

  bool sendMessage(const WebsocketMessage& message);
  bool sendFrame(WebsocketFrame& frame);

private:
  static void static_handle_read(WebsocketConnectionWeakPtr weak_this, const char* begin, const char* end);
  void handle_read(const char* begin, const char* end);
  HttpConnectionPtr connection_;

  void set_message_handler(MessageHandler& handler);
  MessageHandler handler_;

  WebsocketFrame frame_;
  WebsocketMessage message_;
  WebsocketFrameParser frame_parser_;
  WebsocketFrameBuffer frame_buffer_;

  friend WebsocketHttpRequestHandler;
};

}

#endif
