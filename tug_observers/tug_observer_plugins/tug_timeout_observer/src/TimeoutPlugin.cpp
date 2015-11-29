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

#include <tug_timeout_observer/TimeoutPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_yaml/ProcessYaml.h>

namespace tug_observer_plugins_cpp
{

    TimeoutPlugin::TimeoutPlugin() : ObserverPluginBase("timeout")
    { }

    void TimeoutPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      ROS_DEBUG_STREAM("TimeoutPlugin::initialize called");
      if (!params.hasMember("topics"))
      {
        ROS_ERROR("No topics for timeout plugin defined");
        throw std::runtime_error("No topics for timeout plugin defined");
      }
      XmlRpc::XmlRpcValue topics_params = params["topics"];
      for (int i = 0; i < topics_params.size(); ++i)
        subs_.push_back(boost::make_shared<TimeoutSubs>(topics_params[i], this));
    }
}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::TimeoutPlugin, tug_observers::ObserverPluginBase)
