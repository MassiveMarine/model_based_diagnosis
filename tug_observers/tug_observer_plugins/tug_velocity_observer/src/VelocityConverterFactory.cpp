//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterFactory.h>
#include <tug_velocity_observer/VelocityConverterTwist.h>
#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <tug_velocity_observer/VelocityConverterIMU.h>
#include <tug_velocity_observer/VelocityConverterOdometry.h>
#include <tug_velocity_observer/VelocityConverterPoseStamped.h>
#include <tug_velocity_observer/VelocityConverterTf.h>
#include <stdexcept>

boost::shared_ptr<VelocityConverter> VelocityConverterFactory::createVelocityConverter(std::string type, XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base)
{
  if(type == "twist")
    return boost::make_shared<VelocityConverterTwist>(params, call_back, plugin_base);
  else if(type == "twist_stamped")
    return boost::make_shared<VelocityConverterTwistStamped>(params, call_back, plugin_base);
  else if(type == "imu")
    return boost::make_shared<VelocityConverterIMU>(params, call_back, plugin_base);
  else if(type == "odometry")
    return boost::make_shared<VelocityConverterOdometry>(params, call_back, plugin_base);
  else if(type == "pose_stamped")
    return boost::make_shared<VelocityConverterPoseStamped>(params, call_back, plugin_base);
  else if(type == "tf")
    return boost::make_shared<VelocityConverterTf>(params, call_back, plugin_base);
  else
    throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
}
