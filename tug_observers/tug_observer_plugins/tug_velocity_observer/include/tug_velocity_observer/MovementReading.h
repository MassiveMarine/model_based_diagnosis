//
// Created by clemens on 11.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYREADING_H
#define TUG_VELOCITY_OBSERVER_VELOCITYREADING_H

#include <ros/time.h>
#include <sensor_msgs/Imu.h>

struct LinearAccelerationReading
{
    double x;
    double y;
    double z;
};

struct AngularVelocityReading
{
    double x;
    double y;
    double z;
};

struct MovementReading
{
    ros::Time reading_time;
    ros::Time plot_time;
    LinearAccelerationReading linear;
    AngularVelocityReading angular;

    sensor_msgs::Imu toIMUMsg()
    {
      sensor_msgs::Imu result;
      result.header.stamp = plot_time;
      result.angular_velocity.x = angular.x;
      result.angular_velocity.y = angular.y;
      result.angular_velocity.z = angular.z;
      result.linear_acceleration.x = linear.x;
      result.linear_acceleration.y = linear.y;
      result.linear_acceleration.z = linear.z;

      return result;
    }
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYREADING_H
