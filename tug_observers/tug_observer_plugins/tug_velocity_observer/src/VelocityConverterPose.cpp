//
// Created by clemens on 12.09.15.
//

#include <tug_velocity_observer/VelocityConverterPose.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

VelocityConverterPose::VelocityConverterPose(boost::function<void (MovementReading)> call_back) : VelocityConverterPoseStamped(call_back)
{ }

VelocityConverterPose::VelocityConverterPose(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverterPoseStamped(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterPose::PoseCB, this);
}

void VelocityConverterPose::PoseCB(const geometry_msgs::Pose& msg)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose = msg;

  PoseStampedCB(pose_stamped);
}