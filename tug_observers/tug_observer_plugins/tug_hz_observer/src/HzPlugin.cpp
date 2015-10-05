//
// Created by clemens on 22.09.15.
//

#include <tug_hz_observer/HzPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_yaml/ProcessYaml.h>


namespace tug_observer_plugins_cpp
{

    HzPlugin::HzPlugin() : ObserverPluginBase("hz")
    { }

    void HzPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      if (!params.hasMember("topics"))
      {
        ROS_ERROR("No topics for hz plugin defined");
        throw std::runtime_error("No topics for hz plugin defined");
      }
      XmlRpc::XmlRpcValue topics_params = params["topics"];
      for (int i = 0; i < topics_params.size(); ++i)
        subs_.push_back(boost::make_shared<HzSubs>(topics_params[i], this));

      double main_loop_rate = ProcessYaml::getValue<double>("main_loop_rate", params, 1.0);

      timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1./main_loop_rate * 1000. * 1000.), boost::bind(&HzPlugin::run, this));
    }

    void HzPlugin::run()
    {
        for(std::vector<boost::shared_ptr<HzSubs> >::iterator it = subs_.begin(); it != subs_.end(); ++it)
          (*it)->sendResourceInfo();

        flush();
    }
}

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::HzPlugin, tug_observers::ObserverPluginBase)