/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <tug_score_observer/ScoreBase.h>
#include <string>
#include <vector>

namespace tug_observer_plugins_cpp
{
    ScoreBase::ScoreBase(std::string name, XmlRpc::XmlRpcValue params,
                         tug_observers::ObserverPluginBase *plugin_base) :  name_(name)
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
      for (std::vector<ScoreState>::iterator it = states_.begin(); it != states_.end(); ++it)
        if (it->conformsState(filter_state))
          states.push_back(Observation(it->getName(), it->getNumber()));

      if (states.empty())
        ROS_ERROR_STREAM("no state fit for score result " << filter_state);

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
}  // namespace tug_observer_plugins_cpp
