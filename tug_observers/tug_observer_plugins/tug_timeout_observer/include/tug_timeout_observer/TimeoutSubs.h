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
#ifndef TUG_TIMEOUT_OBSERVER_TIMEOUTSUBS_H
#define TUG_TIMEOUT_OBSERVER_TIMEOUTSUBS_H

#include <XmlRpcValue.h>
#include <tug_observers/ObserverPluginBase.h>
#include <map>
#include <string>
#include <tug_timeout_observer/TimeoutBase.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <set>

class TimeoutSubs
{
    std::string topic_;
    ros::Subscriber sub_;
    boost::mutex bases_lock_;
    std::map<std::string, boost::shared_ptr<TimeoutBase> > callerids_config_;
    boost::shared_ptr<TimeoutBase> default_base_;
    tug_observers::ObserverPluginBase* plugin_base_;

public:
    TimeoutSubs(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);
    void cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);
};


#endif  // TUG_TIMEOUT_OBSERVER_TIMEOUTSUBS_H
