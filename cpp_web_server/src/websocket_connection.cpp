#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <limits>
#include "cpp_web_server/websocket_connection.hpp"

namespace cpp_web_server
{

WebsocketConnection::WebsocketConnection(HttpConnectionPtr connection)
  : connection_(connection) {}

void WebsocketConnection::set_message_handler(MessageHandler& handler) {
  handler_ = handler;
}

bool WebsocketConnection::sendMessage(const WebsocketMessage& message){
  WebsocketFrame frame;
  if(frame.fromMessage(message)){
    return sendFrame(frame);
  }
  std::cout << "encode failed"<< std::endl;
  return false;
}

bool WebsocketConnection::sendFrame(WebsocketFrame& frame){
  std::vector<unsigned char> buffer;
  if(frame.serialize(buffer)){
    connection_->write_and_clear(buffer);
    return true;
  }
  std::cout << "send failed"<< std::endl;
  return false;
}


void WebsocketConnection::static_handle_read(WebsocketConnectionWeakPtr weak_this, const char* begin, const char* end) {
  WebsocketConnectionPtr _this = weak_this.lock();
  if(_this)
    _this->handle_read(begin, end);
}
void WebsocketConnection::handle_read(const char* begin, const char* end) {
  boost::tribool frame_result;
  const char* parse_end = begin;
  while(parse_end < end){
    boost::tie(frame_result, parse_end) = frame_parser_.parse(frame_, parse_end, end);
    if(frame_result){
      frame_parser_.reset();
      boost::tribool message_result = frame_buffer_.consume(message_, frame_);

      std::cout << "Frame: size: " << frame_.content.size() << ", opcode: " << frame_.header.opcode << ", fin: " << frame_.header.fin << std::endl;
      if(message_result){
	std::cout << "Read Message: " << message_.type << ", size: " << message_.content.size() << std::endl;
	if(handler_)
	  handler_(message_);
      }
    }
    else if(!frame_result){
      frame_parser_.reset();
      message_.type = WebsocketMessage::type_unknown;
      std::cout << "Failed: " << std::endl;
    }
  }
  connection_->async_read(boost::bind(&WebsocketConnection::static_handle_read, WebsocketConnectionWeakPtr(shared_from_this()), _1, _2));
}

}
