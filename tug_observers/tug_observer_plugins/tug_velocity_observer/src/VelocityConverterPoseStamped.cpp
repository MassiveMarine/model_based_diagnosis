//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterPoseStamped.h>
#include <tug_yaml/ProcessYaml.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

VelocityConverterPoseStamped::VelocityConverterPoseStamped(boost::function<void (MovementReading)> call_back) : VelocityConverterTwistStamped(call_back)
{ }

VelocityConverterPoseStamped::VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base) : VelocityConverterTwistStamped(call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterPoseStamped::PoseStampedCB, this);
}

void VelocityConverterPoseStamped::PoseStampedCB(const geometry_msgs::PoseStamped& msg)
{
  ROS_DEBUG_STREAM("got pose stamped callback with position x:" << msg.pose.position.x << " y:" << msg.pose.position.y << " z:" << msg.pose.position.z
  << " at time sec:" << msg.header.stamp.sec << " nsec:" << msg.header.stamp.nsec);

  if(has_old_position_)
  {
    // transform the new position as a relative position to the new position
    tf::Quaternion old_rotation(old_position_.pose.orientation.x, old_position_.pose.orientation.y, old_position_.pose.orientation.z, old_position_.pose.orientation.w);
    tf::Vector3 old_translation(old_position_.pose.position.x, old_position_.pose.position.y, old_position_.pose.position.z);
    tf::Transform old_pose_as_transformation(old_rotation, old_translation);

    tf::Quaternion new_rotation(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    tf::Vector3 new_translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Transform new_pose_as_transformation(new_rotation, new_translation);

    tf::Transform difference_between = old_pose_as_transformation.inverseTimes(new_pose_as_transformation);

    // devide the transformation by time as this was the velocity to move between these two positions
    double time_difference = (msg.header.stamp - old_position_.header.stamp).toSec();

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header = msg.header;
    twist_msg.twist.linear.x = difference_between.getOrigin().x() / time_difference;
    twist_msg.twist.linear.y = difference_between.getOrigin().y() / time_difference;
    twist_msg.twist.linear.z = difference_between.getOrigin().z() / time_difference;

    tf::Matrix3x3 difference_rot(difference_between.getRotation());
    double roll, pitch, yaw;
    difference_rot.getRPY(roll, pitch, yaw);
    twist_msg.twist.angular.x = roll / time_difference;
    twist_msg.twist.angular.y = pitch / time_difference;
    twist_msg.twist.angular.z = yaw / time_difference;

    TwistStampedCB(twist_msg);
  }

  old_position_ = msg;
  has_old_position_ = true;
}

std::string VelocityConverterPoseStamped::getName()
{
  return topic_;
}