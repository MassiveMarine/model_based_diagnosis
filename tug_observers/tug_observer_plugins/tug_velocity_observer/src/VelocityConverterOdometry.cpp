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

#include <tug_velocity_observer/VelocityConverterOdometry.h>
#include <tug_yaml/ProcessYaml.h>
#include <string>

VelocityConverterOdometry::VelocityConverterOdometry(XmlRpc::XmlRpcValue params,
                                                     boost::function<void(MovementReading)> call_back,
                                                     SubscriberFacade *plugin_base) : VelocityConverterPoseStamped(
        params, call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterOdometry::OdometryCB, this);

  std::string name = getName();
  if (name.find('/') == 0)
    name = name.substr(1, name.size());
  if (name.find_last_of('/') == (name.size() - 1))
    name = name.substr(0, name.size() - 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(name + "_twist", 10);
  movement_pub_ = nh_.advertise<sensor_msgs::Imu>(name + "_movement", 10);
}

void VelocityConverterOdometry::OdometryCB(const nav_msgs::Odometry &msg)
{
  ROS_DEBUG_STREAM("got odom call back");
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg.header;
  pose_stamped.pose = msg.pose.pose;
  PoseStampedCB(pose_stamped);
}

std::string VelocityConverterOdometry::getName()
{
  return topic_;
}
