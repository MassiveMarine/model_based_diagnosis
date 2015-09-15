//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

VelocityConverterTwistStamped::VelocityConverterTwistStamped(boost::function<void (MovementReading)> call_back) : VelocityConverter(call_back)
{ }

VelocityConverterTwistStamped::VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverter(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterTwistStamped::TwistStampedCB, this);
}

void VelocityConverterTwistStamped::TwistStampedCB(const geometry_msgs::TwistStamped& msg)
{
  ROS_DEBUG_STREAM("got twist stamped callback with velocity linear x:" << msg.twist.linear.x << " y:" << msg.twist.linear.y << " z:" << msg.twist.linear.z
                   << " angular x:" << msg.twist.angular.x << " y:" << msg.twist.angular.y << " z:" << msg.twist.angular.z
                   << " at time sec:" << msg.header.stamp.sec << " nsec:" << msg.header.stamp.nsec);

  x_acceleration_calc_.addValue(msg.twist.linear.x, msg.header.stamp);
  y_acceleration_calc_.addValue(msg.twist.linear.y, msg.header.stamp);
  z_acceleration_calc_.addValue(msg.twist.linear.z, msg.header.stamp);

  if(x_acceleration_calc_.hasDifferentiation() && y_acceleration_calc_.hasDifferentiation() && z_acceleration_calc_.hasDifferentiation())
  {
    MovementReading reading;
    reading.reading_time = msg.header.stamp;
    reading.linear.x = x_acceleration_calc_.getDifferentiation();
    reading.linear.y = y_acceleration_calc_.getDifferentiation();
    reading.linear.z = z_acceleration_calc_.getDifferentiation();
    reading.angular.x = msg.twist.angular.x;
    reading.angular.y = msg.twist.angular.y;
    reading.angular.z = msg.twist.angular.z;

    sendMovement(reading);
  }
}