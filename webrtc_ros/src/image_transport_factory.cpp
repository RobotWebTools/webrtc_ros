#include <webrtc_ros/image_transport_factory.h>

namespace webrtc_ros
{

ImageTransportFactory::ImageTransportFactory(rclcpp::Node::SharedPtr node, std::shared_ptr<image_transport::ImageTransport> it)
: node_(node)
, data_(std::make_shared<Data>(it))
{
}

ImageTransportFactory::Subscriber ImageTransportFactory::subscribe (const std::string& topic, const Callback& cb, const std::string& transport)
{
    std::lock_guard<std::mutex> lock{data_->state_mutex_};
    auto it = data_->topics_.find(topic);
    std::shared_ptr<Dispatcher> d;
    if (it != data_->topics_.end())
    {
        d = it->second.lock();
    }
    
    if (!d)  // Either never subscribed, or all subscribers have been destroyed
    {
        d = std::make_shared<Dispatcher>(node_, data_->it_, topic, transport);
        data_->topics_[topic] = d;
    }
    return Subscriber(cb, d);
}

ImageTransportFactory::Dispatcher::Dispatcher(rclcpp::Node::SharedPtr node, std::shared_ptr<image_transport::ImageTransport>& it, const std::string& topic, const std::string& transport)
: next_id_(1)
, node_(node)
{
    image_transport::TransportHints hints(node.get());
    sub_ = it->subscribe(topic, 10, &Dispatcher::dispatch, this, &hints);
    RCLCPP_INFO(node_->get_logger(), "Creating [%s] image_transport for [%s]", transport.c_str(), topic.c_str());
}

ImageTransportFactory::Dispatcher::~Dispatcher()
{
    RCLCPP_INFO(node_->get_logger(),"Destroying [%s] image_transport for [%s]", sub_.getTransport().c_str(), sub_.getTopic().c_str());
    if (callbacks_.size() > 0)
    {
        RCLCPP_ERROR(node_->get_logger(),"BUG in ImageTransportFactory: %zu orphaned subscriber(s)", callbacks_.size());
    }
}

ImageTransportFactory::ID ImageTransportFactory::Dispatcher::addCallback (Callback cb)
{
    std::lock_guard<std::mutex> lock{cb_mutex_};
    ID id = next_id_++;
    callbacks_[id] = cb;
    RCLCPP_INFO(node_->get_logger(),"Creating new callback %u for [%s]", id, sub_.getTopic().c_str());
    return id;
}

void ImageTransportFactory::Dispatcher::removeCallback (ID id)
{
    std::lock_guard<std::mutex> lock{cb_mutex_};
    auto it = callbacks_.find(id);
    if (it != callbacks_.end())
    {
        RCLCPP_INFO(node_->get_logger(),"Destroying callback %u for [%s]", id, sub_.getTopic().c_str());
        callbacks_.erase(it);
    }
}

void ImageTransportFactory::Dispatcher::dispatch (const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock{cb_mutex_};
    for (auto it : callbacks_)
    {
        it.second(msg);
    }
}

ImageTransportFactory::Subscriber::Subscriber (Callback cb, const std::shared_ptr<Dispatcher>& d)
: data_(std::make_shared<Data>())
{
    data_->d_ = d;
    data_->id_ = data_->d_->addCallback(cb);
}

void ImageTransportFactory::Subscriber::shutdown()
{
    if (data_)
    {
        if (data_->d_)
            data_->d_->removeCallback(data_->id_);
        data_->d_.reset();
        data_.reset();
    }
}

} // webrtc_ros
