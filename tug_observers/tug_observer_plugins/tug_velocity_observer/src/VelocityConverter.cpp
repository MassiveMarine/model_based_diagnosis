//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverter.h>

VelocityConverter::VelocityConverter(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back) : call_back_(call_back)
{
  if (params.hasMember("x_filter"))
  {
    XmlRpc::XmlRpcValue x_filter_params = params["x_filter"];
    x_filter_ = boost::make_shared<Filter<double> >(x_filter_params);
  }

  if (params.hasMember("y_filter"))
  {
    XmlRpc::XmlRpcValue y_filter_params = params["y_filter"];
    y_filter_ = boost::make_shared<Filter<double> >(y_filter_params);
  }

  if (params.hasMember("z_filter"))
  {
    XmlRpc::XmlRpcValue z_filter_params = params["z_filter"];
    z_filter_ = boost::make_shared<Filter<double> >(z_filter_params);
  }

  if (params.hasMember("rot_x_filter"))
  {
    ROS_DEBUG("use rot_x_filter for velocity converter");
    XmlRpc::XmlRpcValue rot_x_filter_params = params["rot_x_filter"];
    rot_x_filter_ = boost::make_shared<Filter<double> >(rot_x_filter_params);
  }

  if (params.hasMember("rot_y_filter"))
  {
    ROS_DEBUG("use rot_y_filter for velocity converter");
    XmlRpc::XmlRpcValue rot_y_filter_params = params["rot_y_filter"];
    rot_y_filter_ = boost::make_shared<Filter<double> >(rot_y_filter_params);
  }

  if (params.hasMember("rot_z_filter"))
  {
    ROS_DEBUG("use rot_z_filter for velocity converter");
    XmlRpc::XmlRpcValue rot_z_filter_params = params["rot_z_filter"];
    rot_z_filter_ = boost::make_shared<Filter<double> >(rot_z_filter_params);
  }
}

void VelocityConverter::sendMovement(MovementReading twist)
{
  MovementReading call_back_twist;
  if(x_filter_)
  {
    x_filter_->update(twist.linear.x);
    call_back_twist.linear.x = x_filter_->getFilteState().value;
  }
  else
    call_back_twist.linear.x = twist.linear.x;

  if(y_filter_)
  {
    y_filter_->update(twist.linear.y);
    call_back_twist.linear.y = y_filter_->getFilteState().value;
  }
  else
    call_back_twist.linear.y = twist.linear.y;

  if(z_filter_)
  {
    z_filter_->update(twist.linear.z);
    call_back_twist.linear.z = z_filter_->getFilteState().value;
  }
  else
    call_back_twist.linear.z = twist.linear.z;

  if(rot_x_filter_)
  {
    rot_x_filter_->update(twist.angular.x);
    call_back_twist.angular.x = rot_x_filter_->getFilteState().value;
  }
  else
    call_back_twist.angular.x = twist.angular.x;

  if(rot_y_filter_)
  {
    rot_y_filter_->update(twist.angular.y);
    call_back_twist.angular.y = rot_y_filter_->getFilteState().value;
  }
  else
    call_back_twist.angular.y = twist.angular.y;

  if(rot_z_filter_)
  {
    rot_z_filter_->update(twist.angular.z);
    call_back_twist.angular.z = rot_z_filter_->getFilteState().value;
  }
  else
    call_back_twist.angular.z = twist.angular.z;

  call_back_(call_back_twist);
}
