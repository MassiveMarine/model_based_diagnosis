//
// Created by clemens on 03.08.15.
//

#include <tug_observers_cpp/ObserverPluginBase.h>
#include <tug_observers_msgs/observer_error.h>
#include <tug_observers_cpp/ObserverInfoSender.h>

namespace tug_observers_cpp
{

    ObserverPluginBase::ObserverPluginBase(std::string type) : spinner_(1, &internal_call_back_queue_), type_(type)
    {
      error_pub_ = nh_.advertise<tug_observers_msgs::observer_error>("/observers/error", 1);
    }

    ObserverPluginBase::~ObserverPluginBase()
    {
      spinner_.stop();
    }

    void ObserverPluginBase::startPlugin()
    {
      spinner_.start();
    }

    void ObserverPluginBase::reportError(std::string resource, std::string error_msg, std::string verbose_error_msg, uint32_t error_code)
    {
      ObserverInfoSender::removeInfo(type_, resource);
      tug_observers_msgs::observer_error msg;
      msg.type = type_;
      msg.resource = resource;
      msg.error_msg.error_msg = error_msg;
      msg.error_msg.verbose_error_msg = verbose_error_msg;
      msg.error_msg.error = error_code;
      error_pub_.publish(msg);
    }

    void ObserverPluginBase::reportStates(std::string resource, std::vector<std::string> states)
    {
      ObserverInfoSender::sendInfo(type_, resource, states);
    }
}