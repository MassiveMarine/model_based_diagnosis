//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERFACTORY_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERFACTORY_H

#include <XmlRpc.h>
#include <string>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <tug_velocity_observer/VelocityConverter.h>
#include <tug_velocity_observer/MovementReading.h>
#include <tug_observers/ObserverPluginBase.h>

class VelocityConverterFactory
{
public:
    static boost::shared_ptr<VelocityConverter> createVelocityConverter(std::string type, XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base);
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERFACTORY_H
