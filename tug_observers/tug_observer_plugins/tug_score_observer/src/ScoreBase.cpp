//
// Created by clemens on 30.10.15.
//

#include <tug_score_observer/ScoreBase.h>

namespace tug_observer_plugins_cpp
{
    ScoreBase::ScoreBase(std::string name, XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase *plugin_base) :  name_(name)
    {
      ROS_DEBUG("[ScoreBase::ScoreBase] 1");
      if (!params.hasMember("states"))
      {
        ROS_DEBUG("No nodes given for resource plugin");
        throw std::runtime_error("No nodes given for resource plugin");
      }
      ROS_DEBUG("[ScoreBase::ScoreBase] 2");
      XmlRpc::XmlRpcValue states = params["states"];
      for (int i = 0; i < states.size(); ++i)
      {
        ROS_DEBUG("[ScoreBase::ScoreBase] 2.1");
        XmlRpc::XmlRpcValue &param = states[i];
        ROS_DEBUG("[ScoreBase::ScoreBase] 2.2");
        states_.push_back(ScoreState(param));
      }

      ROS_DEBUG("[ScoreBase::ScoreBase] 3");
      if (!params.hasMember("filter"))
      {
        ROS_DEBUG("No filter for node given for score plugin");
        throw std::runtime_error("No filter node given for score plugin");
      }
      ROS_DEBUG("[ScoreBase::ScoreBase] 4");
      XmlRpc::XmlRpcValue filter_params = params["filter"];
      ROS_DEBUG("[ScoreBase::ScoreBase] 5");
      score_filter_ = boost::make_shared<Filter<double> >(filter_params);

      ROS_DEBUG("[ScoreBase::ScoreBase] 6");
      std::string resource_topic = ProcessYaml::getValue<std::string>("name", params);
      resource_sub_ = plugin_base->subscribe(resource_topic, 1, &ScoreBase::socresCallback, this);
    }

    void ScoreBase::socresCallback(const std_msgs::Float64::ConstPtr &msg)
    {
      boost::mutex::scoped_lock the_lock(class_mutex_);
      ROS_DEBUG_STREAM("got score update callbacke with msg " << msg->data);
      score_filter_->update(msg->data);
      ROS_DEBUG("ScoresPlugin::socresCallback 2");
      last_time_ = ros::Time::now();
    }

    std::vector<Observation> ScoreBase::estimateStates()
    {
      boost::mutex::scoped_lock the_lock(class_mutex_);
      FilteState<double> filter_state = score_filter_->getFilteState();

      std::vector<Observation> states;
      for(std::vector<ScoreState>::iterator it = states_.begin(); it != states_.end(); ++it)
        if(it->conformsState(filter_state))
          states.push_back(Observation(it->getName(), it->getNumber()));

      return states;
    }

    std::string ScoreBase::getName()
    {
      return name_;
    }

    ros::Time ScoreBase::getLastTime()
    {
      boost::mutex::scoped_lock the_lock(class_mutex_);
      return last_time_;
    }
}
