#include <ros/ros.h>
#include <ros/package.h>
#include <webrtc_ros/webrtc_web_server.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <async_web_server_cpp/http_reply.hpp>
#include <async_web_server_cpp/websocket_request_handler.hpp>
#include <async_web_server_cpp/http_server.hpp>
#include <async_web_server_cpp/websocket_connection.hpp>

namespace webrtc_ros
{

SignalingChannel::~SignalingChannel() {}

class SignalingChannelImpl : public SignalingChannel {
public:
  SignalingChannelImpl(async_web_server_cpp::WebsocketConnectionPtr websocket) : websocket_(websocket) {}
  void sendPingMessage() {
    websocket_->sendPingMessage();
  }
  void sendTextMessage(const std::string& message) {
    websocket_->sendTextMessage(message);
  }
private:
  async_web_server_cpp::WebsocketConnectionPtr websocket_;
};

MessageHandler::MessageHandler() {}
MessageHandler::~MessageHandler() {}

WebrtcWebServer::WebrtcWebServer() {}
WebrtcWebServer::~WebrtcWebServer() {}

class WebrtcWebServerImpl : public WebrtcWebServer
{
public:
WebrtcWebServerImpl(int port, SignalingChannelCallback callback, void* data)
  : handler_group_(async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)),
    callback_(callback), data_(data)
{
  std::vector<async_web_server_cpp::HttpHeader> any_origin_headers;
  any_origin_headers.push_back(async_web_server_cpp::HttpHeader("Access-Control-Allow-Origin", "*"));
  handler_group_.addHandlerForPath("/", async_web_server_cpp::HttpReply::from_file(async_web_server_cpp::HttpReply::ok, "text/html",
				   ros::package::getPath("webrtc_ros") + "/web/index.html", any_origin_headers));
  handler_group_.addHandlerForPath("/list_streams.json", boost::bind(&WebrtcWebServerImpl::handle_list_streams, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/viewer", async_web_server_cpp::HttpReply::from_file(async_web_server_cpp::HttpReply::ok, "text/html",
				   ros::package::getPath("webrtc_ros") + "/web/viewer.html", any_origin_headers));
  handler_group_.addHandlerForPath("/webrtc", async_web_server_cpp::WebsocketHttpRequestHandler(boost::bind(&WebrtcWebServerImpl::handle_webrtc_websocket, this, _1, _2)));
  handler_group_.addHandlerForPath("/.+", async_web_server_cpp::HttpReply::from_filesystem(async_web_server_cpp::HttpReply::ok,
											   "/", ros::package::getPath("webrtc_ros") + "/web",
											   false, any_origin_headers));
  server_.reset(new async_web_server_cpp::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port),
                boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4), 1));
}

~WebrtcWebServerImpl()
{
  stop();
}


void run()
{
  server_->run();
  ROS_INFO("Waiting For connections");
}

void stop()
{
  server_->stop();
}

private:

  class WebsocketMessageHandlerWrapper {
  public:
    WebsocketMessageHandlerWrapper(MessageHandler* callback) : callback_(callback) {}
    void operator()(const async_web_server_cpp::WebsocketMessage& message) {
      MessageHandler::Type type;
      if(message.type == async_web_server_cpp::WebsocketMessage::type_text) {
	type = MessageHandler::TEXT;
      }
      else if(message.type == async_web_server_cpp::WebsocketMessage::type_pong) {
	type = MessageHandler::PONG;
      }
      else if(message.type == async_web_server_cpp::WebsocketMessage::type_close) {
	type = MessageHandler::CLOSE;
      }
      else {
	ROS_WARN_STREAM("Unexpected websocket message type: " << message.type << ": " << message.content);
	return;
      }
      callback_->handle_message(type, message.content);
    }
  private:
    boost::shared_ptr<MessageHandler> callback_;
  };

async_web_server_cpp::WebsocketConnection::MessageHandler handle_webrtc_websocket(const async_web_server_cpp::HttpRequest& request,
										  async_web_server_cpp::WebsocketConnectionPtr websocket)
{
  ROS_INFO_STREAM("Handling new WebRTC websocket");
  return WebsocketMessageHandlerWrapper(callback_(data_, new SignalingChannelImpl(websocket)));
}

bool handle_list_streams(const async_web_server_cpp::HttpRequest &request,
    async_web_server_cpp::HttpConnectionPtr connection, const char* begin, const char* end)
{
  std::string image_message_type = ros::message_traits::datatype<sensor_msgs::Image>();
  std::string camera_info_message_type = ros::message_traits::datatype<sensor_msgs::CameraInfo>();

  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it;
  std::vector<std::string> image_topics;
  std::vector<std::string> camera_info_topics;
  for (it = topics.begin(); it != topics.end(); ++it)
  {
    const ros::master::TopicInfo &topic = *it;
    if (topic.datatype == image_message_type)
    {
      image_topics.push_back(topic.name);
    }
    else if (topic.datatype == camera_info_message_type)
    {
      camera_info_topics.push_back(topic.name);
    }
  }

  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "webrtc_ros_server")
  .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("Pragma", "no-cache")
  .header("Content-type", "text/json")
  .write(connection);

  // Don't use jsoncpp cause we link against c++11 library (many not actually be an issue)
  std::stringstream json;
  json << "{\n\t\"camera_topics\": {";
  bool first_cam = true;
  BOOST_FOREACH(const std::string& camera_info_topic, camera_info_topics)
  {
    if (boost::algorithm::ends_with(camera_info_topic, "/camera_info"))
    {
      std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size() - strlen("camera_info"));
      if (!first_cam) json << ",";
      first_cam = false;
      json << "\n\t\t\"" << base_topic << "\": {\n";
      bool first = true;
      std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
      for (; image_topic_itr != image_topics.end();)
      {
        if (boost::starts_with(*image_topic_itr, base_topic))
        {
          if (!first) json << ",\n";
          first = false;
          json << "\t\t\t\"" << image_topic_itr->substr(base_topic.size()) << "\": \"" << *image_topic_itr << "\"";
          image_topic_itr = image_topics.erase(image_topic_itr);
        }
        else
        {
          ++image_topic_itr;
        }
      }
      json << "\n\t\t}";
    }
  }
  json << "\n\t},\n\t\"image_topics\": [\n";
  std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
  for (; image_topic_itr != image_topics.end();)
  {
    json << "\t\t\"" << *image_topic_itr << "\"";
    ++image_topic_itr;
    if(image_topic_itr != image_topics.end()) json << ",";
    json << "\n";
  }
  json << "\t]\n}\n";
  connection->write(json.str());
  return true;
}

static bool ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
                                  const async_web_server_cpp::HttpRequest &request,
                                  async_web_server_cpp::HttpConnectionPtr connection,
                                  const char* begin, const char* end)
{
  ROS_INFO_STREAM(connection->socket().remote_endpoint() << ": " << request.method << " " << request.uri);
  try
  {
    return forward(request, connection, begin, end);
  }
  catch (std::exception &e)
  {
    ROS_WARN_STREAM("Error Handling Request: " << e.what());
  }
  return false;
}



  boost::shared_ptr<async_web_server_cpp::HttpServer> server_;
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_;

  SignalingChannelCallback callback_;
  void* data_;
};





WebrtcWebServer* WebrtcWebServer::create(int port, SignalingChannelCallback callback, void* data) {
  return new WebrtcWebServerImpl(port, callback, data);
}


}
