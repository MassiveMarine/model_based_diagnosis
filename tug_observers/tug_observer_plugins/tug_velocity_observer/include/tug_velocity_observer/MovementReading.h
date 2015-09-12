//
// Created by clemens on 11.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYREADING_H
#define TUG_VELOCITY_OBSERVER_VELOCITYREADING_H

#include <ros/time.h>

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
    LinearAccelerationReading linear;
    AngularVelocityReading angular;
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYREADING_H
