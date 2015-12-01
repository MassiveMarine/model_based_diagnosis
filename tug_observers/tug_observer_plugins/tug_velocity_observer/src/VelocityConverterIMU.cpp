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

#include <tug_velocity_observer/VelocityConverterIMU.h>
#include <tug_yaml/ProcessYaml.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <string>

VelocityConverterIMU::VelocityConverterIMU(XmlRpc::XmlRpcValue params,
                                           boost::function<void(MovementReading)> call_back,
                                           SubscriberFacade *plugin_base) : VelocityConverter(params, call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  imu_sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterIMU::IMUCB, this);
  graviation_cancelation_ = ProcessYaml::getValue<bool>("gravitation_cancelation", params, false);
}

void VelocityConverterIMU::IMUCB(const sensor_msgs::Imu &msg)
{
  MovementReading reading;
  reading.reading_time = msg.header.stamp;
  if (!graviation_cancelation_)
  {
    reading.linear.x = msg.linear_acceleration.x;
    reading.linear.y = msg.linear_acceleration.y;
    reading.linear.z = msg.linear_acceleration.z;
  }
  else
  {
    // gravity cancelation following
    // http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus
    /*   # compensate the accelerometer readings from gravity.
       # @param q the quaternion representing the orientation of a 9DOM MARG sensor array
       # @param acc the readings coming from an accelerometer expressed in g
       #
       # @return a 3d vector representing dinamic acceleration expressed in g
       def gravity_compensate(q, acc):
         g = [0.0, 0.0, 0.0]

         # get expected direction of gravity
         g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
         g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
         g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]

         # compensate accelerometer readings with the expected direction of gravity
         return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]*/

    double g_x = 2. * (msg.orientation.x * msg.orientation.z - msg.orientation.w * msg.orientation.y);
    double g_y = 2. * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z);
    double g_z = msg.orientation.w * msg.orientation.w - msg.orientation.x * msg.orientation.x -
                 msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z;
    reading.linear.x = msg.linear_acceleration.x - g_x * GRAVITY;
    reading.linear.y = msg.linear_acceleration.y - g_y * GRAVITY;
    reading.linear.z = msg.linear_acceleration.z - g_z * GRAVITY;
  }
  reading.angular.x = msg.angular_velocity.x;
  reading.angular.y = msg.angular_velocity.y;
  reading.angular.z = msg.angular_velocity.z;

  sendMovement(reading);
}

std::string VelocityConverterIMU::getName()
{
  return topic_;
}
