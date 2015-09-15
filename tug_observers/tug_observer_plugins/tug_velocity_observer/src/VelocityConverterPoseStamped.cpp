//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterPoseStamped.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

VelocityConverterPoseStamped::VelocityConverterPoseStamped(boost::function<void (MovementReading)> call_back) : VelocityConverterTwistStamped(call_back)
{ }

VelocityConverterPoseStamped::VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverterTwistStamped(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterPoseStamped::PoseStampedCB, this);
}

void VelocityConverterPoseStamped::PoseStampedCB(const geometry_msgs::PoseStamped& msg)
{
  ROS_DEBUG_STREAM("got pose stamped callback with position x:" << msg.pose.position.x << " y:" << msg.pose.position.y << " z:" << msg.pose.position.z
  << " at time sec:" << msg.header.stamp.sec << " nsec:" << msg.header.stamp.nsec);

  linear_x_velocity_calc_.addValue(msg.pose.position.x, msg.header.stamp);
  linear_y_velocity_calc_.addValue(msg.pose.position.y, msg.header.stamp);
  linear_z_velocity_calc_.addValue(msg.pose.position.z, msg.header.stamp);

  tf::Quaternion quat_rot(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 mat_rot(quat_rot);
  double roll, pitch, yaw;
  mat_rot.getRPY(roll, pitch, yaw);
  angular_x_velocity_calc_.addValue(roll, msg.header.stamp);
  angular_y_velocity_calc_.addValue(pitch, msg.header.stamp);
  angular_z_velocity_calc_.addValue(yaw, msg.header.stamp);

  if(linear_x_velocity_calc_.hasDifferentiation() && linear_y_velocity_calc_.hasDifferentiation() && linear_z_velocity_calc_.hasDifferentiation() && angular_x_velocity_calc_.hasDifferentiation() && angular_y_velocity_calc_.hasDifferentiation() && angular_z_velocity_calc_.hasDifferentiation())
  {
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header = msg.header;
    twist_msg.twist.linear.x = linear_x_velocity_calc_.getDifferentiation();
    twist_msg.twist.linear.y = linear_y_velocity_calc_.getDifferentiation();
    twist_msg.twist.linear.z = linear_z_velocity_calc_.getDifferentiation();
    twist_msg.twist.angular.x = angular_x_velocity_calc_.getDifferentiation();
    twist_msg.twist.angular.y = angular_y_velocity_calc_.getDifferentiation();
    twist_msg.twist.angular.z = angular_z_velocity_calc_.getDifferentiation();

    TwistStampedCB(twist_msg);
  }
}