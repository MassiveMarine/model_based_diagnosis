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

#include <tug_hz_observer/HzMergedBases.h>
#include <ros/ros.h>
#include <tug_observers_msgs/observation.h>
#include <tug_yaml/ProcessYaml.h>
#include <sstream>
#include <string>
#include <vector>

HzMergedBases::HzMergedBases(std::string topic, XmlRpc::XmlRpcValue params,
                             tug_observers::ObserverPluginBase* plugin_base) : plugin_base_(plugin_base)
{
  if (!params.hasMember("callerid"))
    throw std::invalid_argument("no caller id given for hz plugin");

  std::stringstream name_stream;
  name_stream << topic << " " << params["callerid"];
  name_ = name_stream.str();

  if (!params.hasMember("states"))
  {
    ROS_ERROR("No states for velocity plugin defined");
    throw std::runtime_error("No states for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue state_params = params["states"];
  for (int i = 0; i < state_params.size(); ++i)
    states_.push_back(HzState(state_params[i]));
}

FilteState<double> HzMergedBases::getHzFilterState()
{
  FilteState<double> result;

  current_filter_time_ = ros::Time::now();

  for (std::vector<boost::shared_ptr<HzBase> >::iterator it = bases_.begin(); it != bases_.end(); ++it)
  {
    FilteState<double> tmp_result = (*it)->getFilterState();
    if (it == bases_.begin())
      result = tmp_result;
    else
    {
      result.value = (result.value * tmp_result.value) / (result.value + tmp_result.value);
      if (result.deviation.size() != tmp_result.deviation.size())
        throw std::runtime_error("different filter sizes for a topic, callerid combination");

      for (size_t i = 0; i < result.deviation.size(); ++i)
        result.deviation[i] += tmp_result.deviation[i];

      result.sample_size += tmp_result.sample_size;
    }
  }

  return result;
}

void HzMergedBases::sendResourceInfo()
{
  if (bases_.empty())  // no topic received
    return;

  ROS_DEBUG("estimate states");
  FilteState<double> hz_state = getHzFilterState();
  if (hz_state.sample_size < 2)
    return;

  std::vector<Observation> states;
  for (std::vector<HzState>::iterator it = states_.begin(); it != states_.end(); ++it)
  {
    ROS_DEBUG_STREAM("check state: '" << it->getName() << "'");
    if (it->conformsState(hz_state))
      states.push_back(Observation(it->getName(), it->getNumber()));
  }
  if (states.empty())
  {
    ROS_ERROR_STREAM("got hz filter state " << hz_state << " for " << name_);
    plugin_base_->reportError(name_, "no_state_" + name_, "For the topic with the name '" + name_
                                                          + "' no state could be estimated",
                              tug_observers_msgs::observation::NO_STATE_FITS, current_filter_time_);
  }
  else
  {
    plugin_base_->reportStates(name_, states, current_filter_time_);
    ROS_DEBUG("HzMergedBases::run 3.1");
  }
}

void HzMergedBases::addBase(boost::shared_ptr<HzBase> base)
{
  bases_.push_back(base);
}
