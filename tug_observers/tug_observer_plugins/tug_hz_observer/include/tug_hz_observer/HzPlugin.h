//
// Created by clemens on 22.09.15.
//

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
}

#endif //TUG_HZ_OBSERVER_HZPLUGIN_H
