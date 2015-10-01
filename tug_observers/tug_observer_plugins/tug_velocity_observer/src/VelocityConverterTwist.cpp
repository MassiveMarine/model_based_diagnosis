//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterTwist.h>
#include <tug_yaml/ProcessYaml.h>

VelocityConverterTwist::VelocityConverterTwist(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base) : VelocityConverterTwistStamped(params, call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterTwist::TwistCB, this);

  std::string name = getName();
  if(name.find('/') == 0)
    name = name.substr(1, name.size());
  if(name.find_last_of('/') == (name.size() - 1))
    name = name.substr(0, name.size() - 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(name + "_twist", 10);
  movement_pub_ = nh_.advertise<sensor_msgs::Imu>(name + "_movement", 10);
}

void VelocityConverterTwist::TwistCB(const geometry_msgs::Twist& msg)
{
  ROS_DEBUG_STREAM("got twist callback with velocity linear x:" << msg.linear.x << " y:" << msg.linear.y << " z:" << msg.linear.z
                   << " angular x:" << msg.angular.x << " y:" << msg.angular.y << " z:" << msg.angular.z);
  geometry_msgs::TwistStamped twist_stamped;
  twist_stamped.header.stamp = ros::Time::now();
  twist_stamped.twist = msg;

  TwistStampedCB(twist_stamped);
}

std::string VelocityConverterTwist::getName()
{
  return topic_;
}