#ifndef CPP_WEB_SERVER_WEBSOCKET_REQUEST_HANDLER_HPP
#define CPP_WEB_SERVER_WEBSOCKET_REQUEST_HANDLER_HPP

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "cpp_web_server/http_request_handler.hpp"
#include "cpp_web_server/websocket_connection.hpp"

namespace cpp_web_server
{

class WebsocketConnection;

typedef boost::function<WebsocketConnection::MessageHandler (const HttpRequest &, boost::shared_ptr<WebsocketConnection>)> WebsocketRequestHandler;

class WebsocketHttpRequestHandler
{
public:
  WebsocketHttpRequestHandler(WebsocketRequestHandler handler);
  void operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end);

  static const std::string KEY_MAGIC_STRING;
private:
  WebsocketRequestHandler handler_;
};

}

#endif
