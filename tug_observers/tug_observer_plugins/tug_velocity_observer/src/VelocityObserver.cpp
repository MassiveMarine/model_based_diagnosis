/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tug_velocity_observer/VelocityObserver.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_velocity_observer/VelocityConverterFactory.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <utility>
#include <string>
#include <vector>

VelocityObserver::VelocityObserver(XmlRpc::XmlRpcValue params, SubscriberFacade *plugin_base) : spinner_(1)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  if (!params.hasMember("source_A"))
  {
    ROS_ERROR("no topic for input A defined");
    throw std::invalid_argument("no topic for input A defined");
  }
  XmlRpc::XmlRpcValue source_a_params = params["source_A"];
  a_input_ = VelocityConverterFactory::createVelocityConverter(
          ProcessYaml::getValue<std::string>("type", source_a_params), source_a_params,
          boost::bind(&VelocityObserver::addTwistA, this, _1), plugin_base);

  if (!params.hasMember("source_B"))
  {
    ROS_ERROR("no topic for input B defined");
    throw std::invalid_argument("no topic for input B defined");
  }
  XmlRpc::XmlRpcValue source_b_params = params["source_B"];
  b_input_ = VelocityConverterFactory::createVelocityConverter(
          ProcessYaml::getValue<std::string>("type", source_b_params), source_b_params,
          boost::bind(&VelocityObserver::addTwistB, this, _1), plugin_base);

  if (params.hasMember("x_filter"))
  {
    XmlRpc::XmlRpcValue x_filter_params = params["x_filter"];
    diff_x_filter_ = boost::make_shared<Filter<double> >(x_filter_params);
  }

  if (params.hasMember("y_filter"))
  {
    XmlRpc::XmlRpcValue y_filter_params = params["y_filter"];
    diff_y_filter_ = boost::make_shared<Filter<double> >(y_filter_params);
  }

  if (params.hasMember("z_filter"))
  {
    XmlRpc::XmlRpcValue z_filter_params = params["z_filter"];
    diff_z_filter_ = boost::make_shared<Filter<double> >(z_filter_params);
  }

  if (params.hasMember("rot_x_filter"))
  {
    XmlRpc::XmlRpcValue rot_x_filter_params = params["rot_x_filter"];
    diff_rot_x_filter_ = boost::make_shared<Filter<double> >(rot_x_filter_params);
  }
  ROS_DEBUG("check for rot_y_filter");
  if (params.hasMember("rot_y_filter"))
  {
    XmlRpc::XmlRpcValue rot_y_filter_params = params["rot_y_filter"];
    diff_rot_y_filter_ = boost::make_shared<Filter<double> >(rot_y_filter_params);
  }
  ROS_DEBUG("check for rot_z_filter");
  if (params.hasMember("rot_z_filter"))
  {
    XmlRpc::XmlRpcValue rot_z_filter_params = params["rot_z_filter"];
    diff_rot_z_filter_ = boost::make_shared<Filter<double> >(rot_z_filter_params);
  }
  ROS_DEBUG("ini states");
  if (!params.hasMember("states"))
  {
    ROS_ERROR("No states for velocity plugin defined");
    throw std::runtime_error("No states for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue state_params = params["states"];
  for (int i = 0; i < state_params.size(); ++i)
    states_.push_back(VelocityState(state_params[i]));

  std::string a_name = a_input_->getName();
  if (a_name.find('/') == 0)
    a_name = a_name.substr(1, a_name.size());
  if (a_name.find_last_of('/') == (a_name.size() - 1))
    a_name = a_name.substr(0, a_name.size() - 1);

  std::string b_name = b_input_->getName();
  if (b_name.find('/') == 0)
    b_name = b_name.substr(1, b_name.size());
  if (b_name.find_last_of('/') == (b_name.size() - 1))
    b_name = b_name.substr(0, b_name.size() - 1);

  a_publisher_ = nh_.advertise<sensor_msgs::Imu>("/" + a_name + "_a_calc_imu", 10);
  b_publisher_ = nh_.advertise<sensor_msgs::Imu>("/" + b_name + "_b_calc_imu", 10);
  a_paired_publisher_ = nh_.advertise<sensor_msgs::Imu>("/" + a_name + "_a_calc_imu_pair", 10);
  b_paired_publisher_ = nh_.advertise<sensor_msgs::Imu>("/" + b_name + "_b_calc_imu_pair", 10);
  filter_result_publisher_ = nh_.advertise<sensor_msgs::Imu>("/" + a_name + "_" + b_name + "_filter_imu", 10);
}

MovementReading VelocityObserver::getCompensatedTwist(MovementReading value)
{
  MovementReading result;
  result.reading_time = value.reading_time;
  result.plot_time = value.plot_time;
  if (diff_x_filter_)
    result.linear.x = value.linear.x;
  else
    result.linear.x = 0.;

  if (diff_y_filter_)
    result.linear.y = value.linear.y;
  else
    result.linear.y = 0.;

  if (diff_z_filter_)
    result.linear.z = value.linear.z;
  else
    result.linear.z = 0.;

  if (diff_rot_x_filter_)
    result.angular.x = value.angular.x;
  else
    result.angular.x = 0.;

  if (diff_rot_y_filter_)
    result.angular.y = value.angular.y;
  else
    result.angular.y = 0.;

  if (diff_rot_z_filter_)
    result.angular.z = value.angular.z;
  else
    result.angular.z = 0.;

  return result;
}

void VelocityObserver::updateFilters(MovementReading a, MovementReading b)
{
  ROS_DEBUG("update filters");
  if (diff_x_filter_)
  {
    ROS_DEBUG_STREAM("update x filter with a: " << a.linear.x << " and b: " << b.linear.x);
    diff_x_filter_->update(a.linear.x - b.linear.x);
  }

  if (diff_y_filter_)
    diff_y_filter_->update(a.linear.y - b.linear.y);

  if (diff_z_filter_)
    diff_z_filter_->update(a.linear.z - b.linear.z);

  if (diff_rot_x_filter_)
    diff_rot_x_filter_->update(a.angular.x - b.angular.x);

  if (diff_rot_y_filter_)
    diff_rot_y_filter_->update(a.angular.y - b.angular.y);

  if (diff_rot_z_filter_)
    diff_rot_z_filter_->update(a.angular.z - b.angular.z);

  if (a.reading_time < b.reading_time)
    current_filter_time_ = b.reading_time;
  else
    current_filter_time_ = a.reading_time;

  a_paired_publisher_.publish(a.toIMUMsg());
  b_paired_publisher_.publish(b.toIMUMsg());
  ROS_DEBUG_STREAM("reading time sec:" << current_filter_time_.sec << ", nsec:" << current_filter_time_.nsec);
}

void VelocityObserver::addTwistA(MovementReading value)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM(
          "add twist of A with accelerations along the axis x:" << value.linear.x << " y:" << value.linear.y <<
          " z:" << value.linear.z << " velocities around the axis x:" << value.angular.x << " y:" <<
          value.angular.y << " z:" << value.angular.z);
  MovementReading compensated_a = getCompensatedTwist(value);
  compensated_a.plot_time = ros::Time(ros::WallTime::now().toSec());
  a_twists_.push_back(compensated_a);
  a_publisher_.publish(compensated_a.toIMUMsg());

  while (!b_twists_.empty())
  {
    MovementReading b_twist = b_twists_.front();
    if (b_twist.reading_time > value.reading_time)
    {
      ROS_DEBUG_STREAM("break during poping b reading time sec:" << b_twist.reading_time.sec << ", nsec:" <<
                       b_twist.reading_time.nsec << " a reading time sec:" << value.reading_time.sec << ", nsec:" <<
                       value.reading_time.nsec);
      break;
    }

    b_twists_.pop_front();
    updateFilters(compensated_a, b_twist);
  }
}

void VelocityObserver::addTwistB(MovementReading value)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM(
          "add twist of B with accelerations along the axis x:" << value.linear.x << " y:" << value.linear.y <<
          " z:" << value.linear.z << " velocities around the axis x:" << value.angular.x << " y:" <<
          value.angular.y << " z:" << value.angular.z);
  MovementReading compensated_b = getCompensatedTwist(value);
  compensated_b.plot_time = ros::Time(ros::WallTime::now().toSec());
  b_twists_.push_back(compensated_b);
  b_publisher_.publish(compensated_b.toIMUMsg());

  while (!a_twists_.empty())
  {
    MovementReading a_twist = a_twists_.front();
    if (a_twist.reading_time > value.reading_time)
      break;

    a_twists_.pop_front();
    updateFilters(a_twist, compensated_b);
  }
}

std::pair<bool, std::vector<Observation> > VelocityObserver::estimateStates()
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM("estimate states");
  FilteState<double> x_state;
  if (diff_x_filter_)
  {
    x_state = diff_x_filter_->getFilteState();
    ROS_DEBUG_STREAM("x filter result " << x_state);
  }

  FilteState<double> y_state;
  if (diff_y_filter_)
  {
    y_state = diff_y_filter_->getFilteState();
    ROS_DEBUG_STREAM("y filter result " << y_state);
  }

  FilteState<double> z_state;
  if (diff_z_filter_)
  {
    z_state = diff_z_filter_->getFilteState();
    ROS_DEBUG_STREAM("z filter result " << z_state);
  }

  FilteState<double> rot_x_state;
  if (diff_rot_x_filter_)
  {
    rot_x_state = diff_rot_x_filter_->getFilteState();
    ROS_DEBUG_STREAM("rot x filter result " << rot_x_state);
  }

  FilteState<double> rot_y_state;
  if (diff_rot_y_filter_)
  {
    rot_y_state = diff_rot_y_filter_->getFilteState();
    ROS_DEBUG_STREAM("rot y filter result " << rot_y_state);
  }

  FilteState<double> rot_z_state;
  if (diff_rot_z_filter_)
  {
    rot_z_state = diff_rot_z_filter_->getFilteState();
    ROS_DEBUG_STREAM("rot z filter result " << rot_z_state);
  }

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time(ros::WallTime::now().toSec());
  if (diff_x_filter_)
    imu_msg.linear_acceleration.x = x_state.value;
  if (diff_y_filter_)
    imu_msg.linear_acceleration.y = y_state.value;
  if (diff_z_filter_)
    imu_msg.linear_acceleration.z = z_state.value;

  if (diff_rot_x_filter_)
    imu_msg.angular_velocity.x = rot_x_state.value;
  if (diff_rot_y_filter_)
    imu_msg.angular_velocity.y = rot_y_state.value;
  if (diff_rot_z_filter_)
    imu_msg.angular_velocity.z = rot_z_state.value;

  filter_result_publisher_.publish(imu_msg);

  ROS_DEBUG_STREAM("itterate through: " << states_.size() << " states");
  std::vector<Observation> result;
  for (std::vector<VelocityState>::iterator it = states_.begin(); it != states_.end(); ++it)
  {
    ROS_DEBUG_STREAM("VelocityObserver::estimateStates 1.1");
    ROS_DEBUG_STREAM("check state: '" << it->getName() << "'");
    if (diff_x_filter_)
    {
      if (!it->canCheckX(x_state))
        return std::make_pair(false, std::vector<Observation>());

      if (!it->conformsStateX(x_state))
      {
        ROS_ERROR_STREAM("x filter is not confrom result " << x_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("VelocityObserver::estimateStates 1.2");
    if (diff_y_filter_)
    {
      if (!it->canCheckY(y_state))
        return std::make_pair(false, std::vector<Observation>());

      if (!it->conformsStateY(y_state))
      {
        ROS_ERROR_STREAM("y filter is not confrom result " << y_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("VelocityObserver::estimateStates 1.3");
    if (diff_z_filter_)
    {
      if (!it->canCheckZ(z_state))
        return std::make_pair(false, std::vector<Observation>());
      if (!it->conformsStateZ(z_state))
      {
        ROS_ERROR_STREAM("z filter is not confrom result " << z_state);
        continue;
      }
    }

    if (diff_rot_x_filter_)
    {
      if (!it->canCheckRotX(rot_x_state))
        return std::make_pair(false, std::vector<Observation>());

      if (!it->conformsStateRotX(rot_x_state))
      {
        ROS_ERROR_STREAM("rot x filter is not confrom result " << rot_x_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("VelocityObserver::estimateStates 1.2");
    if (diff_rot_y_filter_)
    {
      if (!it->canCheckRotY(rot_y_state))
        return std::make_pair(false, std::vector<Observation>());

      if (!it->conformsStateRotY(rot_y_state))
      {
        ROS_ERROR_STREAM("rot y filter is not confrom result " << rot_y_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("VelocityObserver::estimateStates 1.3");
    if (diff_rot_z_filter_)
    {
      if (!it->canCheckRotZ(rot_z_state))
        return std::make_pair(false, std::vector<Observation>());
      if (!it->conformsStateRotZ(rot_z_state))
      {
        ROS_ERROR_STREAM("rot z filter is not confrom result " << rot_z_state);
        continue;
      }
    }

    result.push_back(Observation(it->getName(), it->getNumber()));
  }

  return std::make_pair(true, result);
}

ros::Time VelocityObserver::getCurrentFilterTime()
{
  return current_filter_time_;
}

std::string VelocityObserver::getName()
{
  std::string a_name = a_input_->getName();

  std::string b_name = b_input_->getName();

  std::string result = a_name + " " + b_name;
  return result;
}

std::string VelocityObserver::getInputAName()
{
  return a_input_->getName();
}

std::string VelocityObserver::getInputBName()
{
  return b_input_->getName();
}
