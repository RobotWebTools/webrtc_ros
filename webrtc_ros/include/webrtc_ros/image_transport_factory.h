#ifndef WEBRTC_ROS_IMAGE_TRANSPORT_FACTORY_H_
#define WEBRTC_ROS_IMAGE_TRANSPORT_FACTORY_H_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/transport_hints.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boost/function.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <atomic>

namespace webrtc_ros
{

/* Only subscribe once to each image topic even with multiple listeners */
class ImageTransportFactory
{
private:
    using ID = unsigned int;
    class Dispatcher;
public:
    using Callback = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr& msg)>;
    class Subscriber
    {
    friend class ImageTransportFactory;
    public:
        Subscriber() {}
        void shutdown();
    private:
        Subscriber(Callback cb, const std::shared_ptr<Dispatcher>& d);
        struct Data
        {
            ~Data() { if(d_) d_->removeCallback(id_); }
            std::shared_ptr<Dispatcher> d_;
            ID id_;
        };
        std::shared_ptr<Data> data_;
    };
    ImageTransportFactory (rclcpp::Node::SharedPtr node, std::shared_ptr<image_transport::ImageTransport> it);
    Subscriber subscribe (const std::string& topic, const Callback& cb, const std::string& transport);

private:
    class Dispatcher
    {
    public:
        Dispatcher (rclcpp::Node::SharedPtr node, std::shared_ptr<image_transport::ImageTransport>& it, const std::string& topic, const std::string& transport);
        ~Dispatcher();
        ID addCallback (Callback cb);
        void removeCallback (ID id);
    private:
        Dispatcher(const Dispatcher&) = delete;
        void operator=(const Dispatcher&) = delete;
        void dispatch (const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        image_transport::Subscriber sub_;
        std::mutex cb_mutex_;
        ID next_id_;
        std::map<ID, Callback> callbacks_;
        rclcpp::Node::SharedPtr node_;
    };
    struct Data
    {
        Data(std::shared_ptr<image_transport::ImageTransport> it) : it_(it) {}
        std::mutex state_mutex_;
        std::map<std::string, std::weak_ptr<Dispatcher>> topics_;
        std::shared_ptr<image_transport::ImageTransport>  it_;
        rclcpp::Node::SharedPtr node_;
    };
    std::shared_ptr<Data> data_;
    rclcpp::Node::SharedPtr node_;
};


}



#endif
