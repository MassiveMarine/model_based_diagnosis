//
// Created by clemens on 22.09.15.
//

#ifndef TUG_TIMEOUT_OBSERVER_TIMEOUTPLUGIN_H
#define TUG_TIMEOUT_OBSERVER_TIMEOUTPLUGIN_H

#include <tug_observers/ObserverPluginBase.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <vector>
#include <tug_timeout_observer/TimeoutSubs.h>

namespace tug_observer_plugins_cpp
{
    class TimeoutPlugin : public tug_observers::ObserverPluginBase
    {
        std::vector<boost::shared_ptr<TimeoutSubs> > subs_;

    public:
        TimeoutPlugin();
        virtual void initialize(XmlRpc::XmlRpcValue params);
    };
}

#endif //TUG_TIMEOUT_OBSERVER_TIMEOUTPLUGIN_H
