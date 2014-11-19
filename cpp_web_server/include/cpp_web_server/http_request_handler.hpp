#ifndef CPP_WEB_SERVER_HTTP_REQUEST_HANDLER_HPP
#define CPP_WEB_SERVER_HTTP_REQUEST_HANDLER_HPP

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "cpp_web_server/http_request.hpp"

namespace cpp_web_server
{

class HttpConnection;

typedef boost::function<void(const HttpRequest &, boost::shared_ptr<HttpConnection>, const char* begin, const char* end)> HttpServerRequestHandler;

class HttpRequestHandlerGroup
{
public:
  typedef boost::function<bool(const HttpRequest &)> HandlerPredicate;

  HttpRequestHandlerGroup(HttpServerRequestHandler default_handler);

  void addHandlerForPath(const std::string &path_regex, HttpServerRequestHandler handler);

  void addHandler(HandlerPredicate predicate, HttpServerRequestHandler handler);

  void operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end);

private:
  HttpServerRequestHandler default_handler_;
  std::vector<std::pair<HandlerPredicate, HttpServerRequestHandler> > handlers_;
};

}

#endif
