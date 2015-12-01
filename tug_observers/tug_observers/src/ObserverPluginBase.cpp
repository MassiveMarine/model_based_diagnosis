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
#include <tug_observers/ObserverPluginBase.h>
#include <tug_observers/ObserverInfoSender.h>
#include <string>
#include <vector>

namespace tug_observers
{

    ObserverPluginBase::ObserverPluginBase(std::string type) : spinner_(1, &internal_call_back_queue_), type_(type)
    {
      nh_.setCallbackQueue(&internal_call_back_queue_);
    }

    ObserverPluginBase::~ObserverPluginBase()
    {
      spinner_.stop();
    }

    void ObserverPluginBase::startPlugin()
    {
      spinner_.start();
    }

    void ObserverPluginBase::reportError(std::string resource, std::string error_msg, std::string verbose_error_msg,
                                         int32_t error_code, ros::Time time_of_occurence)
    {
      if (error_code >= 0)
      {
        ROS_WARN_STREAM("report error with state which is positive " <<
                                "-> will not be recognized as error -> change signe of error code");
        error_code *= -1;
      }

      Observation observation(error_msg, verbose_error_msg, error_code);
      std::vector<Observation> observations;
      observations.push_back(observation);

      reportStates(resource, observations, time_of_occurence);

      flush();
    }

    void ObserverPluginBase::reportStates(std::string resource, std::vector<Observation> observations,
                                          ros::Time time_of_occurence)
    {
      ObserverInfoSender::sendInfo(resource, type_, observations, time_of_occurence);
    }

    void ObserverPluginBase::flush()
    {
      ObserverInfoSender::flush();
    }
}  // namespace tug_observers
