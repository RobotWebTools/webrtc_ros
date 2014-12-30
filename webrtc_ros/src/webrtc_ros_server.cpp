#include <ros/ros.h>
#include <ros/package.h>
#include <webrtc_ros/webrtc_ros_server.h>
#include <async_web_server_cpp/http_reply.hpp>
#include <async_web_server_cpp/websocket_request_handler.hpp>
#include "webrtc/base/ssladapter.h"
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace webrtc_ros
{

static void ros_connection_logger(async_web_server_cpp::HttpServerRequestHandler forward,
                                  const async_web_server_cpp::HttpRequest &request,
                                  async_web_server_cpp::HttpConnectionPtr connection,
                                  const char* begin, const char* end)
{
  ROS_INFO_STREAM("Handling Request: " << request.uri);
  try
  {
    forward(request, connection, begin, end);
  }
  catch (std::exception &e)
  {
    ROS_WARN_STREAM("Error Handling Request: " << e.what());
  }
}

async_web_server_cpp::WebsocketConnection::MessageHandler WebrtcRosServer::handle_webrtc_websocket(const async_web_server_cpp::HttpRequest& request,
    async_web_server_cpp::WebsocketConnectionPtr websocket)
{
  ROS_INFO_STREAM("Handling new WebRTC websocket");
  boost::shared_ptr<WebrtcClient> client(new WebrtcClient(nh_, websocket));
  // hold a shared ptr until the object is initialized
  client->init();
  clients_.push_back(client);
  return client->createMessageHandler();
}

WebrtcRosServer::WebrtcRosServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh),
    handler_group_(async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found))
{
  rtc::InitializeSSL();

  int port;
  pnh_.param("port", port, 8080);

  std::vector<async_web_server_cpp::HttpHeader> any_origin_headers;
  any_origin_headers.push_back(async_web_server_cpp::HttpHeader("Access-Control-Allow-Origin", "*"));
  handler_group_.addHandlerForPath("/", boost::bind(&WebrtcRosServer::handle_list_streams, this, _1, _2, _3, _4));
  handler_group_.addHandlerForPath("/viewer", async_web_server_cpp::HttpReply::from_file(async_web_server_cpp::HttpReply::ok, "text/html",
				   ros::package::getPath("webrtc_ros") + "/web/viewer.html", any_origin_headers));
  handler_group_.addHandlerForPath("/webrtc_ros.js", async_web_server_cpp::HttpReply::from_file(async_web_server_cpp::HttpReply::ok, "text/javascript",
				   ros::package::getPath("webrtc_ros") + "/web/webrtc_ros.js", any_origin_headers));
  handler_group_.addHandlerForPath("/webrtc", async_web_server_cpp::WebsocketHttpRequestHandler(boost::bind(&WebrtcRosServer::handle_webrtc_websocket, this, _1, _2)));
  server_.reset(new async_web_server_cpp::HttpServer("0.0.0.0", boost::lexical_cast<std::string>(port),
                boost::bind(ros_connection_logger, handler_group_, _1, _2, _3, _4), 1));
}

void WebrtcRosServer::handle_list_streams(const async_web_server_cpp::HttpRequest &request,
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
  .header("Server", "web_video_server")
  .header("Cache-Control", "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("Pragma", "no-cache")
  .header("Content-type", "text/html;")
  .write(connection);

  connection->write("<html>"
                    "<head><title>ROS Image Topic List</title></head>"
                    "<body><h1>Available ROS Image Topics:</h1>");
  connection->write("<ul>");
  BOOST_FOREACH(std::string & camera_info_topic, camera_info_topics)
  {
    if (boost::algorithm::ends_with(camera_info_topic, "/camera_info"))
    {
      std::string base_topic = camera_info_topic.substr(0, camera_info_topic.size() - strlen("camera_info"));
      connection->write("<li>");
      connection->write(base_topic);
      connection->write("<ul>");
      std::vector<std::string>::iterator image_topic_itr = image_topics.begin();
      for (; image_topic_itr != image_topics.end();)
      {
        if (boost::starts_with(*image_topic_itr, base_topic))
        {
          connection->write("<li><a href=\"/viewer?subscribed_video_topic=");
          connection->write(*image_topic_itr);
          connection->write("\">");
          connection->write(image_topic_itr->substr(base_topic.size()));
          connection->write("</a>");
          connection->write("</li>");

          image_topic_itr = image_topics.erase(image_topic_itr);
        }
        else
        {
          ++image_topic_itr;
        }
      }
      connection->write("</ul>");
    }
    connection->write("</li>");
  }
  connection->write("</ul>");
  connection->write("<form action=\"/viewer\">"
                    "Subscribe Video: <input name=\"subscribed_video_topic\" type=\"text\"><br>"
                    "Publish Video: <input name=\"published_video_topic\" type=\"text\"><br>"
                    "<input type=\"submit\" value=\"Go\">"
                    "</form>");

  connection->write("</body></html>");
}


WebrtcRosServer::~WebrtcRosServer()
{
  BOOST_FOREACH(WebrtcClientWeakPtr client_weak, clients_)
  {
    boost::shared_ptr<WebrtcClient> client = client_weak.lock();
    if (client)
      client->invalidate();
  }
  rtc::CleanupSSL();
}

void WebrtcRosServer::run()
{
  server_->run();
  ROS_INFO("Waiting For connections");
}

void WebrtcRosServer::stop()
{
  server_->stop();
}

}
