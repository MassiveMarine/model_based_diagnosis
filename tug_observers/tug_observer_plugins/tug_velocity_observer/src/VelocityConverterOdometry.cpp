//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterOdometry.h>
#include <tug_yaml/ProcessYaml.h>

VelocityConverterOdometry::VelocityConverterOdometry(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverterPoseStamped(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterOdometry::OdometryCB, this);
}

void VelocityConverterOdometry::OdometryCB(const nav_msgs::Odometry& msg)
{
  ROS_DEBUG_STREAM("got odom call back");
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg.header;
  pose_stamped.pose = msg.pose.pose;
  PoseStampedCB(pose_stamped);
}