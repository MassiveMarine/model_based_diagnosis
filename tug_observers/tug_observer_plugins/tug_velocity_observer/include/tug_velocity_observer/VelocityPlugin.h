//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYPLUGIN_H
#define TUG_VELOCITY_OBSERVER_VELOCITYPLUGIN_H

#include <tug_observers/ObserverPluginBase.h>
#include <tug_velocity_observer/VelocityObserver.h>
#include <tug_velocity_observer/VelocityChecker.h>
#include <tug_timers/Timer.h>

namespace tug_observer_plugins_cpp
{
    class VelocityPlugin : public tug_observers::ObserverPluginBase, public VelocityChecker
    {
        boost::shared_ptr<Timer> timer_;
    public:
        VelocityPlugin();

        virtual void initialize(XmlRpc::XmlRpcValue params);

        void run();
    };
}


#endif //TUG_VELOCITY_OBSERVER_VELOCITYPLUGIN_H
