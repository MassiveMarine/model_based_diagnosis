/*
This file is part of the software provided by the tug ais group
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tug_observers/ObserverInfoSender.h>
#include <vector>
#include <string>

ObserverInfoSender::ObserverInfoSender()
{
  ROS_DEBUG_STREAM("constructor of observer info sender called");
  ros::NodeHandle private_nh("~");
  double rate;
  private_nh.param<double>("info_rate", rate, 1.0);
  timeout_thread_ = boost::make_shared<Timeout>(boost::posix_time::milliseconds(1. / rate * 1000.),
                                                boost::bind(&ObserverInfoSender::executeFlush, this));

  info_pub_ = nh_.advertise<tug_observers_msgs::observer_info>("/observers/info", 1);
}

ObserverInfoSender &ObserverInfoSender::getInstance()
{
  static ObserverInfoSender instance;

  return instance;
}

void ObserverInfoSender::sendInfoIntern(std::string resource, std::string type, std::vector<Observation> observations,
                                        ros::Time time_of_occurence)
{
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);

  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = time_of_occurence;
  observation_info.resource = resource;
  observation_info.type = type;

  for (size_t i = 0; i < observations.size(); ++i)
    observation_info.observation.push_back(observations[i].toMsg());

  current_obser_info_.observation_infos.push_back(observation_info);
}

bool ObserverInfoSender::executeFlush()
{
  ROS_DEBUG_STREAM("execute flush called");
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);
  ROS_DEBUG_STREAM("check if there are observations to publish");
  if (current_obser_info_.observation_infos.empty())
    return true;
  ROS_DEBUG_STREAM("publish observer info");
  info_pub_.publish(current_obser_info_);
  current_obser_info_.observation_infos.clear();

  ROS_DEBUG_STREAM("return with true");
  return true;
}

void ObserverInfoSender::flushIntern()
{
  ROS_DEBUG_STREAM("flushIntern");
  timeout_thread_->set();
  executeFlush();
}

void ObserverInfoSender::sendInfo(std::string resource, std::string type, std::vector<Observation> observations,
                                  ros::Time time_of_occurence)
{
  getInstance().sendInfoIntern(resource, type, observations, time_of_occurence);
}

void ObserverInfoSender::flush()
{
  getInstance().flushIntern();
}
