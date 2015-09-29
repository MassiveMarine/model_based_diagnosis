//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERIMU_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERIMU_H

#include <tug_velocity_observer/VelocityConverter.h>
#include <tug_observers/SubscriberFacade.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

class VelocityConverterIMU : public VelocityConverter
{
    const static double GRAVITY=9.81;
    std::string topic_;
    ros::Subscriber imu_sub_;
    bool graviation_cancelation_;
public:
    VelocityConverterIMU(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void IMUCB(const sensor_msgs::Imu& msg);
    virtual std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERIMU_H
