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

#ifndef TUG_OBSERVERS_OBSERVERINFOSENDER_H
#define TUG_OBSERVERS_OBSERVERINFOSENDER_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <tug_time/Timer.h>
#include <tug_observers/Observation.h>
#include <tug_observers_msgs/observer_info.h>
#include <tug_time/Timeout.h>

class ObserverInfoSender
{
  ros::NodeHandle nh_;
  ros::Publisher info_pub_;
  boost::mutex observer_infos_mutex_;
  tug_observers_msgs::observer_info current_obser_info_;
  boost::shared_ptr<Timeout> timeout_thread_;

  ObserverInfoSender();
  void sendInfoIntern(std::string resource, std::string type, std::vector<Observation> observations,
                      ros::Time time_of_occurence);
  void flushIntern();
  bool executeFlush();

  public:
    static ObserverInfoSender& getInstance();
    static void sendInfo(std::string resource, std::string type, std::vector<Observation> observations,
                         ros::Time time_of_occurence);
    static void flush();
};


#endif  // TUG_OBSERVERS_OBSERVERINFOSENDER_H
