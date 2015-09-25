//
// Created by clemens on 24.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCHECKER_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCHECKER_H

#include <tug_observers/ObserverPluginBase.h>
#include <tug_velocity_observer/VelocityObserver.h>

class VelocityChecker
{
    std::vector<boost::shared_ptr<VelocityObserver> > observers_;

protected:
    std::vector<boost::tuple<std::string, std::vector<std::string>, ros::Time> > velocityObservations();
public:
    VelocityChecker();

    void init(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCHECKER_H
