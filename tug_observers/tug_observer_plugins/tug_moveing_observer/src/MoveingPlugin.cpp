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

#include <tug_moveing_observer/MoveingPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>
#include <string>
#include <boost/tuple/tuple.hpp>
#include <vector>

namespace tug_observer_plugins_cpp
{
    MoveingPlugin::MoveingPlugin() : ObserverPluginBase("moveing")
    { }

    void MoveingPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      ROS_DEBUG("MoveingPlugin::initialize 1");
      double rate = ProcessYaml::getValue<double>("loop_rate", params, 1.0);
      timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1. / rate * 1000. * 1000.),
                                         boost::bind(&MoveingPlugin::run, this));
      ROS_DEBUG("MoveingPlugin::init 2");
      if (!params.hasMember("moveings"))
      {
        ROS_ERROR("No correlations for velocity plugin defined");
        throw std::runtime_error("No correlations for velocity plugin defined");
      }
      ROS_DEBUG("MoveingPlugin::init 3");
      XmlRpc::XmlRpcValue moveings_params = params["moveings"];
      for (int i = 0; i < moveings_params.size(); ++i)
        observers_.push_back(boost::make_shared<MoveingObserver>(moveings_params[i], this));
      ROS_DEBUG("MoveingPlugin::init 4");
    }

    std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > MoveingPlugin::velocityObservations()
    {
      std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > result;
      for (std::vector<boost::shared_ptr<MoveingObserver> >::iterator it = observers_.begin();
              it != observers_.end(); ++it)
      {
        std::pair<bool, std::vector<Observation> > states = (*it)->estimateStates();
        if (states.first)
        {
          ROS_DEBUG_STREAM("got estimated states: " << states.second.size());
          result.push_back(
                  boost::make_tuple<std::string, std::vector<Observation>, ros::Time>((*it)->getName(), states.second,
                  (*it)->getCurrentFilterTime()));
        }
        else
          ROS_DEBUG("got no estimated states");
      }

      return result;
    }

    void MoveingPlugin::run()
    {
      ROS_DEBUG("MoveingPlugin::run 1");
      std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > observations =
              velocityObservations();

      ROS_DEBUG("MoveingPlugin::run 2");
      for (size_t i = 0; i < observations.size(); ++i)
      {
        ROS_DEBUG("MoveingPlugin::run 2.1");
        std::string name = observations[i].head;
        std::vector<Observation> states = observations[i].tail.head;
        ros::Time observation_time = observations[i].tail.tail.head;
        ROS_DEBUG("MoveingPlugin::run 2.2");
        if (states.empty())
        {
          reportError(name, "no_state",
                      "For the input pair with the name '" + name + "' no state could be estimated",
                      tug_observers_msgs::observation::NO_STATE_FITS, observation_time);
        }
        else
        {
          reportStates(name, states, observation_time);
        }
      }

      ROS_DEBUG("MoveingPlugin::run 3");
      flush();
    }
}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::MoveingPlugin, tug_observers::ObserverPluginBase)
