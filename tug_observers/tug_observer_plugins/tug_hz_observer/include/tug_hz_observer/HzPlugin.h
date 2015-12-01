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

#ifndef TUG_HZ_OBSERVER_HZPLUGIN_H
#define TUG_HZ_OBSERVER_HZPLUGIN_H

#include <tug_observers/ObserverPluginBase.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <vector>
#include <tug_hz_observer/HzSubs.h>
#include <tug_time/Timer.h>

namespace tug_observer_plugins_cpp
{
class HzPlugin : public tug_observers::ObserverPluginBase
{
    std::vector<boost::shared_ptr<HzSubs> > subs_;
    boost::shared_ptr<Timer> timer_;

public:
    HzPlugin();
    virtual void initialize(XmlRpc::XmlRpcValue params);
    void run();
};
}  // namespace tug_observer_plugins_cpp

#endif  // TUG_HZ_OBSERVER_HZPLUGIN_H
