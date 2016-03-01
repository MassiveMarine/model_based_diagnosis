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
#include <tug_moveing_observer/VelocityConverterPoseStamped.h>
#include <tug_yaml/ProcessYaml.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <string>

VelocityConverterPoseStamped::VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params,
                                                           boost::function<void(MovementReading)> call_back)
        : VelocityConverterTwistStamped(params, call_back), window_size_(5)
{
  window_size_ = ProcessYaml::getValue<unsigned int>("velocity_window_size", params, 5);
  pose_buffer_ = boost::circular_buffer<geometry_msgs::PoseStamped>(window_size_);
  linear_x_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  linear_y_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  linear_z_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  angular_x_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  angular_y_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  angular_z_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
}

VelocityConverterPoseStamped::VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params,
                                                           boost::function<void(MovementReading)> call_back,
                                                           SubscriberFacade *plugin_base)
        : VelocityConverterTwistStamped(params, call_back), window_size_(5)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterPoseStamped::PoseStampedCB, this);

  std::string name = getName();
  if (name.find('/') == 0)
    name = name.substr(1, name.size());
  if (name.find_last_of('/') == (name.size() - 1))
    name = name.substr(0, name.size() - 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(name + "_twist", 10);

  window_size_ = ProcessYaml::getValue<unsigned int>("velocity_window_size", params, 5);
  pose_buffer_ = boost::circular_buffer<geometry_msgs::PoseStamped>(window_size_);
  linear_x_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  linear_y_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  linear_z_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  angular_x_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  angular_y_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
  angular_z_velocity_calc_ = boost::make_shared<SimpleLinearRegression<double> >(window_size_);
}

void VelocityConverterPoseStamped::PoseStampedCB(const geometry_msgs::PoseStamped &msg)
{
  ROS_DEBUG_STREAM(
          "got pose stamped callback with position x:" << msg.pose.position.x << " y:" << msg.pose.position.y <<
          " z:" << msg.pose.position.z
          << " at time sec:" << msg.header.stamp.sec << " nsec:" << msg.header.stamp.nsec);

  pose_buffer_.push_back(msg);

  if (pose_buffer_.size() == window_size_)
  {
    // transform all poses to the first pose in the window to use linear regression calculation on the positions
    tf::Quaternion old_rotation(pose_buffer_[0].pose.orientation.x, pose_buffer_[0].pose.orientation.y,
                                pose_buffer_[0].pose.orientation.z, pose_buffer_[0].pose.orientation.w);
    tf::Vector3 old_translation(pose_buffer_[0].pose.position.x, pose_buffer_[0].pose.position.y,
                                pose_buffer_[0].pose.position.z);
    tf::Transform old_pose_as_transformation(old_rotation, old_translation);
    ROS_DEBUG_STREAM("got old pose stamped callback with position x:" << pose_buffer_[0].pose.position.x << " y:" <<
                     pose_buffer_[0].pose.position.y << " z:" << pose_buffer_[0].pose.position.z
                     << " at time sec:" << pose_buffer_[0].header.stamp.sec << " nsec:" <<
                     pose_buffer_[0].header.stamp.nsec);

    linear_x_velocity_calc_->clear();
    linear_y_velocity_calc_->clear();
    linear_z_velocity_calc_->clear();
    angular_x_velocity_calc_->clear();
    angular_y_velocity_calc_->clear();
    angular_z_velocity_calc_->clear();

    for (size_t i = 0; i < pose_buffer_.size(); ++i)
    {
      tf::Quaternion new_rotation(pose_buffer_[i].pose.orientation.x, pose_buffer_[i].pose.orientation.y,
                                  pose_buffer_[i].pose.orientation.z, pose_buffer_[i].pose.orientation.w);
      tf::Vector3 new_translation(pose_buffer_[i].pose.position.x, pose_buffer_[i].pose.position.y,
                                  pose_buffer_[i].pose.position.z);
      tf::Transform new_pose_as_transformation(new_rotation, new_translation);
      ROS_DEBUG_STREAM("got new pose stamped callback with position x:" << pose_buffer_[i].pose.position.x << " y:" <<
                       pose_buffer_[i].pose.position.y << " z:" << pose_buffer_[i].pose.position.z
                       << " at time sec:" << pose_buffer_[i].header.stamp.sec << " nsec:" <<
                       pose_buffer_[i].header.stamp.nsec);

      tf::Transform difference_between = old_pose_as_transformation.inverseTimes(new_pose_as_transformation);

      linear_x_velocity_calc_->addValue(difference_between.getOrigin().x(), pose_buffer_[i].header.stamp);
      linear_y_velocity_calc_->addValue(difference_between.getOrigin().y(), pose_buffer_[i].header.stamp);
      linear_z_velocity_calc_->addValue(difference_between.getOrigin().z(), pose_buffer_[i].header.stamp);
      tf::Matrix3x3 difference_rot(difference_between.getRotation());
      double roll, pitch, yaw;
      difference_rot.getRPY(roll, pitch, yaw);
      angular_x_velocity_calc_->addValue(roll, pose_buffer_[i].header.stamp);
      angular_y_velocity_calc_->addValue(pitch, pose_buffer_[i].header.stamp);
      angular_z_velocity_calc_->addValue(yaw, pose_buffer_[i].header.stamp);
    }

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header = msg.header;
    twist_msg.twist.linear.x = linear_x_velocity_calc_->getDifferentiation();
    twist_msg.twist.linear.y = linear_y_velocity_calc_->getDifferentiation();
    twist_msg.twist.linear.z = linear_z_velocity_calc_->getDifferentiation();
    twist_msg.twist.angular.x = angular_x_velocity_calc_->getDifferentiation();
    twist_msg.twist.angular.y = angular_y_velocity_calc_->getDifferentiation();
    twist_msg.twist.angular.z = angular_z_velocity_calc_->getDifferentiation();

    TwistStampedCB(twist_msg);
  }
}

std::string VelocityConverterPoseStamped::getName()
{
  return topic_;
}
