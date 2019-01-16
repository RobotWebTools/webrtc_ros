#ifndef WEBRTC_ROS_IMAGE_TRANSPORT_FACTORY_H_
#define WEBRTC_ROS_IMAGE_TRANSPORT_FACTORY_H_

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
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
    using Callback = boost::function<void(const sensor_msgs::ImageConstPtr& msg)>;
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
    ImageTransportFactory (const image_transport::ImageTransport& it);
    Subscriber subscribe (const std::string& topic, const Callback& cb, const std::string& transport);
private:
    class Dispatcher
    {
    public:
        Dispatcher (image_transport::ImageTransport& it, const std::string& topic, const std::string& transport);
        ~Dispatcher();
        ID addCallback (Callback cb);
        void removeCallback (ID id);
    private:
        Dispatcher(const Dispatcher&) = delete;
        void operator=(const Dispatcher&) = delete;
        void dispatch (const sensor_msgs::ImageConstPtr& msg);
        image_transport::Subscriber sub_;
        std::mutex cb_mutex_;
        ID next_id_;
        std::map<ID, Callback> callbacks_;
    };
    struct Data
    {
        Data(const image_transport::ImageTransport& it) : it_(it) {}
        std::mutex state_mutex_;
        std::map<std::string, std::weak_ptr<Dispatcher>> topics_;
        image_transport::ImageTransport it_;
    };
    std::shared_ptr<Data> data_;
};


}



#endif
