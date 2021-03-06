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

#ifndef TUG_OBSERVERS_OBSERVERPLUGINBASE_H
#define TUG_OBSERVERS_OBSERVERPLUGINBASE_H

#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>
#include <ros/ros.h>
#include <tug_observers/SubscriberFacade.h>
#include <tug_observers/Observation.h>
#include <string>
#include <vector>

namespace tug_observers
{
class ObserverPluginBase : public SubscriberFacade
{
private:
  ros::CallbackQueue internal_call_back_queue_;
  ros::AsyncSpinner spinner_;
  std::string type_;
  bool is_started_up_;
  ros::Time start_up_time_;

public:
  virtual ~ObserverPluginBase();

  void startPlugin();

  virtual void initialize(XmlRpc::XmlRpcValue params) = 0;

  void reportError(std::string resource, std::string error_msg, std::string verbose_error_msg,
                   int32_t error_code, ros::Time time_of_occurence);

  void reportStates(std::string resource, std::vector<Observation> observations, ros::Time time_of_occurence);

  void flush();

  void setStartUpTime(double start_up_time);

  bool isStartedUp();

protected:
  explicit ObserverPluginBase(std::string type);
};
}  // namespace tug_observers

#endif  // TUG_OBSERVERS_OBSERVERPLUGINBASE_H
