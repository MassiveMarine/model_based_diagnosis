//
// Created by clemens on 24.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCHECKER_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCHECKER_H

#include <tug_observers/ObserverPluginBase.h>
#include <tug_velocity_observer/VelocityObserver.h>
#include <map>

class VelocityChecker
{
    std::vector<boost::shared_ptr<VelocityObserver> > observers_;

    void updateFaultCount(std::string name, unsigned int increment, std::map<std::string, unsigned  int>& fault_count);

protected:
    std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > velocityObservations();
public:
    VelocityChecker();

    void init(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);

    std::map<std::string, bool> getValidInputs();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCHECKER_H
