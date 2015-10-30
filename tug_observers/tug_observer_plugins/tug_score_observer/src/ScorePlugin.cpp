//
// Created by clemens on 04.08.15.
//

#include <tug_score_observer/ScorePlugin.h>
#include <ros/ros.h>
#include <tug_yaml/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>

namespace tug_observer_plugins_cpp
{
    ScoresPlugin::ScoresPlugin() : ObserverPluginBase("scores")
    {

    }

    void ScoresPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      ROS_DEBUG("[ScoresPlugin::initialize] 1");
      if (!params.hasMember("topics"))
      {
        ROS_DEBUG("No topics given for scores plugin");
        throw std::runtime_error("No topics given for scores plugin");
      }
      ROS_DEBUG("[ScoresPlugin::initialize] 2");
      XmlRpc::XmlRpcValue topics = params["topics"];
      for (int i = 0; i < topics.size(); ++i)
      {
        std::string name = ProcessYaml::getValue<std::string>("name", topics[i]);
        ROS_DEBUG("[ScoresPlugin::initialize] 2.1");
        XmlRpc::XmlRpcValue &param = topics[i];
        ROS_DEBUG("[ScoresPlugin::initialize] 2.2");
        bases_.push_back(boost::make_shared<ScoreBase>(name, param, this));
      }

      double main_loop_rate = ProcessYaml::getValue<double>("main_loop_rate", params, 1.0);

      timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1./main_loop_rate * 1000. * 1000.), boost::bind(&ScoresPlugin::run, this));
    }

    void ScoresPlugin::run()
    {
      for(std::vector<boost::shared_ptr<ScoreBase> >::iterator it = bases_.begin(); it != bases_.end(); ++it)
      {
        ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.3");
        std::string name = (*it)->getName();
        ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.4 " << name);
        std::vector<Observation> states = (*it)->estimateStates();
        ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.5 " << " with number of states: " << states.size());
        if(states.empty())
        {
          reportError(name, "no_state_" + name, "For the node with the name '" + name + "' no state could be estimated", tug_observers_msgs::observation::NO_STATE_FITS, (*it)->getLastTime());
        }
        else
        {
          reportStates(name, states, (*it)->getLastTime());
          ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.5");
        }
      }

      flush();
    }
}

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ScoresPlugin, tug_observers::ObserverPluginBase)