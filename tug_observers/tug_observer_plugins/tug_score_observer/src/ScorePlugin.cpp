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

#include <tug_score_observer/ScorePlugin.h>
#include <ros/ros.h>
#include <tug_yaml/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>
#include <string>
#include <vector>

namespace tug_observer_plugins_cpp
{
    ScoresPlugin::ScoresPlugin() : ObserverPluginBase("scores")
    { }

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

      timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1./main_loop_rate * 1000. * 1000.),
                                         boost::bind(&ScoresPlugin::run, this));
    }

    void ScoresPlugin::run()
    {
      if (!isStartedUp())
        return;

      for (std::vector<boost::shared_ptr<ScoreBase> >::iterator it = bases_.begin(); it != bases_.end(); ++it)
      {
        ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.3");
        std::string name = (*it)->getName();
        ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.4 " << name);
        std::vector<Observation> states = (*it)->estimateStates();
        ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.5 " << " with number of states: " << states.size());
        if (states.empty())
        {
          reportError(name, "no_state_" + name, "For the topic with the name '" + name
                                                + "' no state could be estimated",
                      tug_observers_msgs::observation::NO_STATE_FITS, (*it)->getLastTime());
        }
        else
        {
          reportStates(name, states, (*it)->getLastTime());
          ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.5");
        }
      }

      flush();
    }
}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ScoresPlugin, tug_observers::ObserverPluginBase)
