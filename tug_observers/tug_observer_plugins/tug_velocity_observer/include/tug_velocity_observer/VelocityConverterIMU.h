//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERIMU_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERIMU_H

#include <tug_velocity_observer/VelocityConverter.h>
#include <tug_observers/ObserverPluginBase.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

class VelocityConverterIMU : public VelocityConverter
{
    ros::Subscriber imu_sub_;
public:
    VelocityConverterIMU(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base);
    void IMUCB(const sensor_msgs::Imu& msg);
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERIMU_H
