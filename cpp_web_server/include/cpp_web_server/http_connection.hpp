#ifndef CPP_WEB_SERVER_HTTP_CONNECTION_HPP
#define CPP_WEB_SERVER_HTTP_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include "cpp_web_server/http_request_handler.hpp"
#include "cpp_web_server/http_request.hpp"
#include "cpp_web_server/http_request_parser.hpp"

namespace cpp_web_server
{

class HttpConnection;
typedef boost::shared_ptr<HttpConnection> HttpConnectionPtr;
typedef boost::weak_ptr<HttpConnection> HttpConnectionWeakPtr;

// Represents a connection to a client
// To keep the connection alive keep a shared pointer to this object
class HttpConnection : public boost::enable_shared_from_this<HttpConnection>,
                       private boost::noncopyable
{
public:
  typedef boost::function<void(const char* begin, const char* end)> ReadHandler;
  typedef boost::shared_ptr<void> ResourcePtr;

  explicit HttpConnection(boost::asio::io_service &io_service,
			  HttpServerRequestHandler request_handler);

  boost::asio::ip::tcp::socket &socket();

  // Start async operation to read request
  void start();

  void async_read(ReadHandler callback);

  void write_and_clear(std::vector<unsigned char> &data);

  void write(const std::string &);

  void write(const boost::asio::const_buffer &buffer,
      ResourcePtr resource);

  void write(const std::vector<boost::asio::const_buffer> &buffer,
      ResourcePtr resource);

private:
  void handle_read(const char* begin, const char* end);
  void handle_read_raw(ReadHandler callback,
		       const boost::system::error_code &e,
		       std::size_t bytes_transferred);

  // Must be called while holding write lock
  void write_pending();

  void handle_write(const boost::system::error_code &e,
      std::vector<boost::shared_ptr<void> > resources);

  boost::asio::io_service::strand strand_;
  boost::asio::ip::tcp::socket socket_;
  HttpServerRequestHandler request_handler_;
  boost::array<char, 8192> buffer_;
  HttpRequest request_;
  HttpRequestParser request_parser_;

  boost::mutex write_mutex_;
  bool write_in_progress_;
  std::vector<boost::asio::const_buffer> pending_write_buffers_;
  std::vector<ResourcePtr> pending_write_resources_;
  boost::system::error_code last_error_;
  ReadHandler read_handler_;
};

}

#endif
