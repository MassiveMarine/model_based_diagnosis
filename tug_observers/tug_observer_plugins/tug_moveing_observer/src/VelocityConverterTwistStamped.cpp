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

#include <tug_moveing_observer/VelocityConverterTwistStamped.h>
#include <tug_yaml/ProcessYaml.h>
#include <boost/make_shared.hpp>
#include <tug_observer_plugin_utils/differentiation/SimpleLinearRegression.h>
#include <string>

VelocityConverterTwistStamped::VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params,
                                                             boost::function<void(MovementReading)> call_back)
        : VelocityConverter(params, call_back)
{
}

VelocityConverterTwistStamped::VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params,
                                                             boost::function<void(MovementReading)> call_back,
                                                             SubscriberFacade *plugin_base) :
                                                                              VelocityConverter(params, call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterTwistStamped::TwistStampedCB, this);

  std::string name = getName();
  if (name.find('/') == 0)
    name = name.substr(1, name.size());
  if (name.find_last_of('/') == (name.size() - 1))
    name = name.substr(0, name.size() - 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(name + "_twist", 10);
}

void VelocityConverterTwistStamped::TwistStampedCB(const geometry_msgs::TwistStamped &msg)
{
  ROS_DEBUG_STREAM(
          "got twist stamped callback with velocity linear x:" << msg.twist.linear.x << " y:" << msg.twist.linear.y <<
          " z:" << msg.twist.linear.z
          << " angular x:" << msg.twist.angular.x << " y:" << msg.twist.angular.y << " z:" << msg.twist.angular.z
          << " at time sec:" << msg.header.stamp.sec << " nsec:" << msg.header.stamp.nsec);

  ros::Time plot_time = ros::Time(ros::WallTime::now().toSec());
  geometry_msgs::TwistStamped new_msg;
  new_msg.twist = msg.twist;
  new_msg.header.stamp = plot_time;

  MovementReading reading;
  reading.plot_time = plot_time;
  reading.reading_time = msg.header.stamp;
  reading.linear.x = msg.twist.linear.x;
  reading.linear.y = msg.twist.linear.y;
  reading.linear.z = msg.twist.linear.z;
  reading.angular.x = msg.twist.angular.x;
  reading.angular.y = msg.twist.angular.y;
  reading.angular.z = msg.twist.angular.z;


  twist_pub_.publish(new_msg);

  sendMovement(reading);
}

std::string VelocityConverterTwistStamped::getName()
{
  return topic_;
}
