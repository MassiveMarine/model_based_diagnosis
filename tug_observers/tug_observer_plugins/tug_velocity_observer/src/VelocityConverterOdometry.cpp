//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterOdometry.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

VelocityConverterOdometry::VelocityConverterOdometry(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverterPose(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterOdometry::OdometryCB, this);
}

void VelocityConverterOdometry::OdometryCB(const nav_msgs::Odometry& msg)
{
  PoseCB(msg.pose.pose);
}