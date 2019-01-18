#include <webrtc_ros/image_transport_factory.h>

namespace webrtc_ros
{

ImageTransportFactory::ImageTransportFactory(const image_transport::ImageTransport& it)
: data_(std::make_shared<Data>(it))
{
}

ImageTransportFactory::Subscriber ImageTransportFactory::subscribe (const std::string& topic, const Callback& cb, const std::string& transport)
{
    std::lock_guard<std::mutex> lock{data_->state_mutex_};
    auto it = data_->topics_.find(topic);
    std::shared_ptr<Dispatcher> d;
    if (it != data_->topics_.end()) d = it->second.lock();
    if (!d)  // Either never subscribed, or all subscribers have been destroyed
    {
        d = std::make_shared<Dispatcher>(data_->it_, topic, transport);
        data_->topics_[topic] = d;
    }
    return Subscriber(cb, d);
}

ImageTransportFactory::Dispatcher::Dispatcher(image_transport::ImageTransport& it, const std::string& topic, const std::string& transport)
: sub_(it.subscribe(topic, 1, boost::bind(&Dispatcher::dispatch, this, _1), ros::VoidPtr(), image_transport::TransportHints(transport))),
  next_id_(1)
{
    ROS_INFO("Creating [%s] image_transport for [%s]", transport.c_str(), topic.c_str());
}

ImageTransportFactory::Dispatcher::~Dispatcher()
{
    ROS_INFO("Destroying [%s] image_transport for [%s]", sub_.getTransport().c_str(), sub_.getTopic().c_str());
    if (callbacks_.size() > 0)
        ROS_ERROR("BUG in ImageTransportFactory: %zu orphaned subscriber(s)", callbacks_.size());
}

ImageTransportFactory::ID ImageTransportFactory::Dispatcher::addCallback (Callback cb)
{
    std::lock_guard<std::mutex> lock{cb_mutex_};
    ID id = next_id_++;
    callbacks_[id] = cb;
    ROS_INFO("Creating new callback %u for [%s]", id, sub_.getTopic().c_str());
    return id;
}

void ImageTransportFactory::Dispatcher::removeCallback (ID id)
{
    std::lock_guard<std::mutex> lock{cb_mutex_};
    auto it = callbacks_.find(id);
    if (it != callbacks_.end())
    {
        ROS_INFO("Destroying callback %u for [%s]", id, sub_.getTopic().c_str());
        callbacks_.erase(it);
    }
}

void ImageTransportFactory::Dispatcher::dispatch (const sensor_msgs::ImageConstPtr& msg)
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
