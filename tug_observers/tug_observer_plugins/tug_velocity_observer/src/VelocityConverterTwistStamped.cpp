//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <tug_yaml/ProcessYaml.h>
#include <boost/make_shared.hpp>
#include <tug_observer_plugin_utils/differentiation/SimpleLinearRegression.h>

VelocityConverterTwistStamped::VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back) : VelocityConverter(call_back)
{
  unsigned int window_size = ProcessYaml::getValue<unsigned int>("acceleartion_window_size", params, 7);
  x_acceleration_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size);
  y_acceleration_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size);
  z_acceleration_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size);
}

VelocityConverterTwistStamped::VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base) : VelocityConverter(call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterTwistStamped::TwistStampedCB, this);

  std::string name = getName();
  if(name.find('/') == 0)
    name = name.substr(1, name.size());
  if(name.find_last_of('/') == (name.size() - 1))
    name = name.substr(0, name.size() - 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(name + "_twist", 10);
  movement_pub_ = nh_.advertise<sensor_msgs::Imu>(name + "_movement", 10);

  unsigned int window_size = ProcessYaml::getValue<unsigned int>("acceleartion_window_size", params, 7);
  x_acceleration_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size);
  y_acceleration_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size);
  z_acceleration_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size);
}

void VelocityConverterTwistStamped::TwistStampedCB(const geometry_msgs::TwistStamped& msg)
{
  ROS_DEBUG_STREAM("got twist stamped callback with velocity linear x:" << msg.twist.linear.x << " y:" << msg.twist.linear.y << " z:" << msg.twist.linear.z
                   << " angular x:" << msg.twist.angular.x << " y:" << msg.twist.angular.y << " z:" << msg.twist.angular.z
                   << " at time sec:" << msg.header.stamp.sec << " nsec:" << msg.header.stamp.nsec);

  x_acceleration_calc_->addValue(msg.twist.linear.x, msg.header.stamp);
  y_acceleration_calc_->addValue(msg.twist.linear.y, msg.header.stamp);
  z_acceleration_calc_->addValue(msg.twist.linear.z, msg.header.stamp);

  if(x_acceleration_calc_->hasDifferentiation() && y_acceleration_calc_->hasDifferentiation() && z_acceleration_calc_->hasDifferentiation())
  {
    ros::Time plot_time = ros::Time(ros::WallTime::now().toSec());
    geometry_msgs::TwistStamped new_msg;
    new_msg.twist = msg.twist;
    new_msg.header.stamp = plot_time;

    MovementReading reading;
    reading.plot_time = plot_time;
    reading.reading_time = msg.header.stamp;
    reading.linear.x = x_acceleration_calc_->getDifferentiation();
    reading.linear.y = y_acceleration_calc_->getDifferentiation();
    reading.linear.z = z_acceleration_calc_->getDifferentiation();
    reading.angular.x = msg.twist.angular.x;
    reading.angular.y = msg.twist.angular.y;
    reading.angular.z = msg.twist.angular.z;


    twist_pub_.publish(new_msg);
    movement_pub_.publish(reading.toIMUMsg());

    sendMovement(reading);
  }
}

std::string VelocityConverterTwistStamped::getName()
{
  return topic_;
}