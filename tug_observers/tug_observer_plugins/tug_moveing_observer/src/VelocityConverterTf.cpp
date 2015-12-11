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

#include <tug_moveing_observer/VelocityConverterTf.h>
#include <tug_yaml/ProcessYaml.h>
#include <string>

VelocityConverterTf::VelocityConverterTf(XmlRpc::XmlRpcValue params, boost::function<void(MovementReading)> call_back)
        : VelocityConverterPoseStamped(params, call_back), tf_update_rate_(1.0)
{
  target_frame_ = ProcessYaml::getValue<std::string>("target_frame", params);
  base_frame_ = ProcessYaml::getValue<std::string>("base_frame", params);
}

void VelocityConverterTf::run()
{
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
      tf_listener_.lookupTransform(target_frame_, base_frame_,
                                   ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_STREAM("transformation exception " << ex.what());
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = transform.frame_id_;
    pose_stamped.header.stamp = transform.stamp_;
    pose_stamped.pose.position.x = transform.getOrigin().x();
    pose_stamped.pose.position.y = transform.getOrigin().y();
    pose_stamped.pose.position.z = transform.getOrigin().z();
    pose_stamped.pose.orientation.x = transform.getRotation().x();
    pose_stamped.pose.orientation.y = transform.getRotation().y();
    pose_stamped.pose.orientation.z = transform.getRotation().z();
    pose_stamped.pose.orientation.w = transform.getRotation().w();
    PoseStampedCB(pose_stamped);

    tf_update_rate_.sleep();
  }
}

std::string VelocityConverterTf::getName()
{
  return base_frame_ + "_" + target_frame_;
}
