//
// Created by clemens on 03.08.15.
//

#include <tug_observers_cpp/ObserverPluginBase.h>

namespace tug_observers_cpp
{

    ObserverPluginBase::ObserverPluginBase() : spinner_(1, &internal_call_back_queue_)
    { }

    ObserverPluginBase::~ObserverPluginBase()
    {
      spinner_.stop();
    }

    void ObserverPluginBase::startPlugin()
    {
      spinner_.start();
    }

    void ObserverPluginBase::reportError(std::string resource, std::string error_msg, std::string verbose_error_msg)
    {
        ROS_ERROR_STREAM("report error for resource: '" << resource << "' with msg: '" << error_msg << "' which means in detail '" << verbose_error_msg << "'");
    }

    void ObserverPluginBase::reportStates(std::string resource, std::vector<std::string> states)
    {
      std::string estimated_states = "";
      for(std::vector<std::string>::iterator it = states.begin(); it != states.end(); ++it)
        estimated_states += " '" + *it + "'";
      ROS_INFO_STREAM("report resource: '" << resource << " is one of the following states:" << estimated_states);
    }
}