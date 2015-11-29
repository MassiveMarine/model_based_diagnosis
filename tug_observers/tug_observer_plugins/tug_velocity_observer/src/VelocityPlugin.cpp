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

#include <tug_velocity_observer/VelocityPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>
#include <string>
#include <boost/tuple/tuple.hpp>
#include <vector>

namespace tug_observer_plugins_cpp
{

    VelocityPlugin::VelocityPlugin() : ObserverPluginBase("velocity")
    { }

    void VelocityPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      ROS_DEBUG("VelocityPlugin::initialize 1");
      init(params, this);

      ROS_DEBUG("VelocityPlugin::initialize 2");
      double rate = ProcessYaml::getValue<double>("loop_rate", params, 1.0);
      timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1. / rate * 1000. * 1000.),
                                         boost::bind(&VelocityPlugin::run, this));
      ROS_DEBUG("VelocityPlugin::initialize 3");
    }

    void VelocityPlugin::run()
    {
      ROS_DEBUG("VelocityPlugin::run 1");
      std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > observations =
              velocityObservations();

      ROS_DEBUG("VelocityPlugin::run 2");
      for (size_t i = 0; i < observations.size(); ++i)
      {
        ROS_DEBUG("VelocityPlugin::run 2.1");
        std::string name = observations[i].head;
        std::vector<Observation> states = observations[i].tail.head;
        ros::Time observation_time = observations[i].tail.tail.head;
        ROS_DEBUG("VelocityPlugin::run 2.2");
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

      ROS_DEBUG("VelocityPlugin::run 3");
      flush();
    }
}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::VelocityPlugin, tug_observers::ObserverPluginBase)
