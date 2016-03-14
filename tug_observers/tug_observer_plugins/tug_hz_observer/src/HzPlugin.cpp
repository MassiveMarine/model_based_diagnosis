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

#include <tug_hz_observer/HzPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_yaml/ProcessYaml.h>
#include <vector>

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

  timer_ = boost::make_shared<Timer>(boost::posix_time::microseconds(1./main_loop_rate * 1000. * 1000.),
                                     boost::bind(&HzPlugin::run, this));
}

void HzPlugin::run()
{
    if (!isStartedUp())
      return;

    for (std::vector<boost::shared_ptr<HzSubs> >::iterator it = subs_.begin(); it != subs_.end(); ++it)
      (*it)->sendResourceInfo();

    flush();
}
}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::HzPlugin, tug_observers::ObserverPluginBase)
