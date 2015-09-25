//
// Created by clemens on 22.09.15.
//

#include <tug_timeout_observer/TimeoutPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

namespace tug_observer_plugins_cpp
{

    TimeoutPlugin::TimeoutPlugin() : ObserverPluginBase("timeout")
    { }

    void TimeoutPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      if (!params.hasMember("topics"))
      {
        ROS_ERROR("No topics for timeout plugin defined");
        throw std::runtime_error("No topics for timeout plugin defined");
      }
      XmlRpc::XmlRpcValue topics_params = params["topics"];
      for (int i = 0; i < topics_params.size(); ++i)
        subs_.push_back(boost::make_shared<TimeoutSubs>(topics_params[i], this));

    }
}

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::TimeoutPlugin, tug_observers::ObserverPluginBase)