//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterOdometry.h>
#include <tug_yaml/ProcessYaml.h>

VelocityConverterOdometry::VelocityConverterOdometry(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base) : VelocityConverterPoseStamped(call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterOdometry::OdometryCB, this);
}

void VelocityConverterOdometry::OdometryCB(const nav_msgs::Odometry& msg)
{
  ROS_DEBUG_STREAM("got odom call back");
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg.header;
  pose_stamped.pose = msg.pose.pose;
  PoseStampedCB(pose_stamped);
}

std::string VelocityConverterOdometry::getName()
{
  return topic_;
}