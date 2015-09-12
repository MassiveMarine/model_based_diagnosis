//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterTwist.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

VelocityConverterTwist::VelocityConverterTwist(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverterTwistStamped(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterTwist::TwistCB, this);
}

void VelocityConverterTwist::TwistCB(const geometry_msgs::Twist& msg)
{
  geometry_msgs::TwistStamped twist_stamped;
  twist_stamped.header.stamp = ros::Time::now();
  twist_stamped.twist = msg;

  TwistStampedCB(twist_stamped);
}