//
// Created by clemens on 03.08.15.
//

#include <tug_observers/ObserverPluginBase.h>
#include <tug_observers_msgs/observer_error.h>
#include <tug_observers/ObserverInfoSender.h>

namespace tug_observers
{

    ObserverPluginBase::ObserverPluginBase(std::string type) : spinner_(1, &internal_call_back_queue_), type_(type)
    {
      nh_.setCallbackQueue(&internal_call_back_queue_);
    }

    ObserverPluginBase::~ObserverPluginBase()
    {
      spinner_.stop();
    }

    void ObserverPluginBase::startPlugin()
    {
      spinner_.start();
    }

    void ObserverPluginBase::reportError(std::string resource, std::string error_msg, std::string verbose_error_msg, int32_t error_code, ros::Time time_of_occurence)
    {
      if(error_code >= 0)
      {
        ROS_WARN_STREAM("report error with state which is positive -> will not be recognized as error -> change signe of error code");
        error_code *= -1;
      }

      Observation observation(error_msg,verbose_error_msg, error_code);
      std::vector<Observation> observations;
      observations.push_back(observation);

      reportStates(resource, observations, time_of_occurence);

      flush();
    }

    void ObserverPluginBase::reportStates(std::string resource, std::vector<Observation> observations, ros::Time time_of_occurence)
    {
      ObserverInfoSender::sendInfo(resource, type_, observations, time_of_occurence);
    }

    void ObserverPluginBase::flush()
    {
      ObserverInfoSender::flush();
    }
}
