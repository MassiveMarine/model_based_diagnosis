//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterTf.h>

VelocityConverterTf::VelocityConverterTf(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base);
void VelocityConverterTf::run();