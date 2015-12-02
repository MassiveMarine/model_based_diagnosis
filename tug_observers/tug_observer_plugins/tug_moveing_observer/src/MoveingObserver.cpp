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

#include <tug_moveing_observer/MoveingObserver.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_moveing_observer/VelocityConverterFactory.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <utility>
#include <string>
#include <vector>

MoveingObserver::MoveingObserver(XmlRpc::XmlRpcValue params, SubscriberFacade *plugin_base) : use_roll_(false),
                                                                                                use_pitch_(false),
                                                                                                use_yaw_(false),
                                                                                                spinner_(1)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG("MoveingObserver::MoveingObserver 1: get source parameter");
  if (!params.hasMember("source"))
  {
    ROS_ERROR("no topic for input A defined");
    throw std::invalid_argument("no topic for input A defined");
  }
  XmlRpc::XmlRpcValue source_params = params["source"];
  ROS_DEBUG("MoveingObserver::MoveingObserver 2: create velocity converter");
  input_ = VelocityConverterFactory::createVelocityConverter(
          ProcessYaml::getValue<std::string>("type", source_params), source_params,
          boost::bind(&MoveingObserver::addTwist, this, _1), plugin_base);

  ROS_DEBUG("MoveingObserver::MoveingObserver 3: check for x filter");
  if (params.hasMember("x_filter"))
  {
    XmlRpc::XmlRpcValue x_filter_params = params["x_filter"];
    x_filter_ = boost::make_shared<Filter<double> >(x_filter_params);
  }

  ROS_DEBUG("MoveingObserver::MoveingObserver 4: check for y filter");
  if (params.hasMember("y_filter"))
  {
    XmlRpc::XmlRpcValue y_filter_params = params["y_filter"];
    y_filter_ = boost::make_shared<Filter<double> >(y_filter_params);
  }

  ROS_DEBUG("MoveingObserver::MoveingObserver 5: check for z filter");
  if (params.hasMember("z_filter"))
  {
    XmlRpc::XmlRpcValue z_filter_params = params["z_filter"];
    z_filter_ = boost::make_shared<Filter<double> >(z_filter_params);
  }

  ROS_DEBUG("MoveingObserver::MoveingObserver 6: check if roll pitch and yaw should be used");
  use_roll_ = ProcessYaml::getValue<bool>("use_roll", params, false);
  use_pitch_ = ProcessYaml::getValue<bool>("use_pitch", params, false);
  use_yaw_ = ProcessYaml::getValue<bool>("use_yaw", params, false);

  ROS_DEBUG("MoveingObserver::MoveingObserver 7: check for rotation x filter");
  if (!params.hasMember("rot_x_filter"))
  {
    ROS_ERROR("No rot_x_filter for velocity plugin defined");
    throw std::runtime_error("No rot_x_filter for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue rot_x_filter_params = params["rot_x_filter"];
  rot_x_filter_ = boost::make_shared<Filter<double> >(rot_x_filter_params);

  ROS_DEBUG("MoveingObserver::MoveingObserver 8: check for rotation y filter");
  if (!params.hasMember("rot_y_filter"))
  {
    ROS_ERROR("No rot_y_filter for velocity plugin defined");
    throw std::runtime_error("No rot_y_filter for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue rot_y_filter_params = params["rot_y_filter"];
  rot_y_filter_ = boost::make_shared<Filter<double> >(rot_y_filter_params);

  ROS_DEBUG("MoveingObserver::MoveingObserver 9: check for rotation z filter");
  if (!params.hasMember("rot_z_filter"))
  {
    ROS_ERROR("No rot_z_filter for velocity plugin defined");
    throw std::runtime_error("No rot_z_filter for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue rot_z_filter_params = params["rot_z_filter"];
  rot_z_filter_ = boost::make_shared<Filter<double> >(rot_z_filter_params);

  ROS_DEBUG("MoveingObserver::MoveingObserver 10: check for states");
  if (!params.hasMember("states"))
  {
    ROS_ERROR("No states for velocity plugin defined");
    throw std::runtime_error("No states for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue state_params = params["states"];
  for (int i = 0; i < state_params.size(); ++i)
    states_.push_back(VelocityState(state_params[i]));

  ROS_DEBUG("MoveingObserver::MoveingObserver 11: create publisher for debug message");
  std::string a_name = input_->getName();
  if (a_name.find('/') == 0)
    a_name = a_name.substr(1, a_name.size());
  if (a_name.find_last_of('/') == (a_name.size() - 1))
    a_name = a_name.substr(0, a_name.size() - 1);

  a_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/" + a_name + "_a_calc_imu", 10);
}

MovementReading MoveingObserver::getCompensatedTwist(MovementReading value)
{
  MovementReading result;
  if (x_filter_)
    result.linear.x = value.linear.x;
  else
    result.linear.x = 0.;

  if (y_filter_)
    result.linear.y = value.linear.y;
  else
    result.linear.y = 0.;

  if (z_filter_)
    result.linear.z = value.linear.z;
  else
    result.linear.z = 0.;

  if (use_roll_)
    result.angular.x = value.angular.x;
  else
    result.angular.x = 0.;

  if (use_pitch_)
    result.angular.y = value.angular.y;
  else
    result.angular.y = 0.;

  if (use_yaw_)
    result.angular.z = value.angular.z;
  else
    result.angular.z = 0.;

  return result;
}

void MoveingObserver::updateFilters(MovementReading a)
{
  ROS_DEBUG("update filters");
  if (x_filter_)
  {
    ROS_DEBUG_STREAM("update x filter with a: " << a.linear.x);
    x_filter_->update(a.linear.x);
  }

  if (y_filter_)
    y_filter_->update(a.linear.y);

  if (z_filter_)
    z_filter_->update(a.linear.z);

  rot_x_filter_->update(a.angular.x);
  rot_y_filter_->update(a.angular.y);
  rot_z_filter_->update(a.angular.z);

  current_filter_time_ = a.reading_time;

  ROS_DEBUG_STREAM("reading time sec:" << current_filter_time_.sec << ", nsec:" << current_filter_time_.nsec);
}

void MoveingObserver::addTwist(MovementReading value)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM(
          "add twist of A with accelerations along the axis x:" << value.linear.x << " y:" << value.linear.y <<
          " z:" << value.linear.z << " velocities around the axis x:" << value.angular.x << " y:" <<
          value.angular.y << " z:" << value.angular.z);
  MovementReading compensated_a = getCompensatedTwist(value);
  compensated_a.plot_time = ros::Time(ros::WallTime::now().toSec());
  a_publisher_.publish(compensated_a.toTwistMsg());
  updateFilters(compensated_a);
}

std::pair<bool, std::vector<Observation> > MoveingObserver::estimateStates()
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM("estimate states");
  FilteState<double> x_state;
  if (x_filter_)
  {
    x_state = x_filter_->getFilteState();
    ROS_DEBUG_STREAM("x filter result " << x_state);
  }

  FilteState<double> y_state;
  if (y_filter_)
  {
    y_state = y_filter_->getFilteState();
    ROS_DEBUG_STREAM("y filter result " << y_state);
  }

  FilteState<double> z_state;
  if (z_filter_)
  {
    z_state = z_filter_->getFilteState();
    ROS_DEBUG_STREAM("z filter result " << z_state);
  }

  FilteState<double> rot_x_state = rot_x_filter_->getFilteState();
  ROS_DEBUG_STREAM("rot x filter result " << rot_x_state);

  FilteState<double> rot_y_state = rot_y_filter_->getFilteState();
  ROS_DEBUG_STREAM("rot y filter result " << rot_y_state);

  FilteState<double> rot_z_state = rot_z_filter_->getFilteState();
  ROS_DEBUG_STREAM("rot z filter result " << rot_z_state);

  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time(ros::WallTime::now().toSec());
  if (x_filter_)
    twist_msg.twist.linear.x = x_state.value;
  if (y_filter_)
    twist_msg.twist.linear.y = y_state.value;
  if (z_filter_)
    twist_msg.twist.linear.z = z_state.value;

  twist_msg.twist.angular.x = rot_x_state.value;
  twist_msg.twist.angular.y = rot_y_state.value;
  twist_msg.twist.angular.z = rot_z_state.value;

  ROS_DEBUG_STREAM("itterate through: " << states_.size() << " states");
  std::vector<Observation> result;
  for (std::vector<VelocityState>::iterator it = states_.begin(); it != states_.end(); ++it)
  {
    ROS_DEBUG_STREAM("MoveingObserver::estimateStates 1.1");
    ROS_DEBUG_STREAM("check state: '" << it->getName() << "'");
    if (x_filter_)
    {
      if (!it->canCheckX(x_state))
        return std::make_pair(false, std::vector<Observation>());

      if (!it->conformsStateX(x_state))
      {
        ROS_ERROR_STREAM("x filter is not confrom result " << x_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("MoveingObserver::estimateStates 1.2");
    if (y_filter_)
    {
      if (!it->canCheckY(x_state))
        return std::make_pair(false, std::vector<Observation>());

      if (!it->conformsStateY(y_state))
      {
        ROS_ERROR_STREAM("y filter is not confrom result " << y_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("MoveingObserver::estimateStates 1.3");
    if (z_filter_)
    {
      if (!it->canCheckZ(x_state))
        return std::make_pair(false, std::vector<Observation>());
      if (!it->conformsStateZ(z_state))
      {
        ROS_ERROR_STREAM("z filter is not confrom result " << z_state);
        continue;
      }
    }
    ROS_DEBUG_STREAM("MoveingObserver::estimateStates 1.4");
    if (it->canCheck(rot_x_state, rot_y_state, rot_z_state))
      return std::make_pair(false, std::vector<Observation>());
    if (it->conformsState(rot_x_state, rot_y_state, rot_z_state))
      result.push_back(Observation(it->getName(), it->getNumber()));
    else
    {
      ROS_ERROR_STREAM("rotation filter is not confrom result x:" << rot_x_state << " y:" << rot_y_state << " z:" <<
                       rot_z_state);
    }
  }

  return std::make_pair(true, result);
}

ros::Time MoveingObserver::getCurrentFilterTime()
{
  return current_filter_time_;
}

std::string MoveingObserver::getName()
{
  return input_->getName();
}
