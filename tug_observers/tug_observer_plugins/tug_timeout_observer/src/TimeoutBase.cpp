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

#include <tug_timeout_observer/TimeoutBase.h>
#include <tug_observers_msgs/observation.h>
#include <string>
#include <vector>

TimeoutBase::TimeoutBase(std::string topic, XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base)
        : plugin_base_(plugin_base)
{
  std::stringstream name_stream;
  name_stream << topic << " " << params["callerid"];
  name_ = name_stream.str();

  if (!params.hasMember("timeout"))
  {
    ROS_ERROR("No timeout for timeout plugin defined");
    throw std::runtime_error("No timeout for timeout plugin defined");
  }
  timeout_ = ProcessYaml::getValue<double>("timeout", params);

  if (params.hasMember("max_timeouts_in_a_row"))
  {
    has_max_timeouts_in_a_row_ = true;
    max_timeouts_in_a_row_ = ProcessYaml::getValue<int>("max_timeouts_in_a_row", params);
  }
  else
    has_max_timeouts_in_a_row_ = false;
  remaining_timeouts_ = max_timeouts_in_a_row_;

  ROS_DEBUG_STREAM("init timeout for topic " << topic << " with timeout: " << timeout_);

  timeout_thread_ = boost::make_shared<Timeout>(boost::posix_time::seconds(timeout_),
                                                boost::bind(&TimeoutBase::timeout_callback, this));
}

void TimeoutBase::update()
{
  ROS_DEBUG_STREAM("TimeoutBase::update called");
  std::vector<Observation> observations;
  observations.push_back(Observation("ok", tug_observers_msgs::observation::GENERAL_OK));
  ROS_DEBUG_STREAM("TimeoutBase::update report states");
  plugin_base_->reportStates(name_, observations, ros::Time::now());
  ROS_DEBUG_STREAM("TimeoutBase::update set for timeout call");
  timeout_thread_->set();
  ROS_DEBUG_STREAM("TimeoutBase::update set remaining timouts");
  remaining_timeouts_ = max_timeouts_in_a_row_;
}

bool TimeoutBase::timeout_callback()
{
  if (!plugin_base_->isStartedUp())
    return true;

  if (has_max_timeouts_in_a_row_)
  {
    if (remaining_timeouts_ <= 0)
      return false;

    remaining_timeouts_ -= 1;
  }

  ROS_ERROR_STREAM("timeout for: " << name_);
  plugin_base_->reportError(name_, "no_state_" + name_, "For the topic with the name '" + name_ +
          "' no state could be estimated", tug_observers_msgs::observation::TIMEOUT, ros::Time::now());

  return true;
}
