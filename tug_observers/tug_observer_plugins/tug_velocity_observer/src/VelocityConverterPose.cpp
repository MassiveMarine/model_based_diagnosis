//
// Created by clemens on 12.09.15.
//

#include <tug_velocity_observer/VelocityConverterPose.h>
#include <tug_yaml/ProcessYaml.h>

VelocityConverterPose::VelocityConverterPose(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back) : VelocityConverterPoseStamped(params, call_back)
{ }

VelocityConverterPose::VelocityConverterPose(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base) : VelocityConverterPoseStamped(params, call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterPose::PoseCB, this);
}

void VelocityConverterPose::PoseCB(const geometry_msgs::Pose& msg)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose = msg;

  PoseStampedCB(pose_stamped);
}

std::string VelocityConverterPose::getName()
{
  return topic_;
}