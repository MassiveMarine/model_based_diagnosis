//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWISTSTAMPED_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWISTSTAMPED_H

#include <tug_velocity_observer/VelocityConverter.h>
#include <tug_observers/SubscriberFacade.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tug_observer_plugin_utils/differentiation/SingleSideDifferentiation.h>

class VelocityConverterTwistStamped : public VelocityConverter
{
    std::string topic_;
    SingleSideDifferentiation<double> x_acceleration_calc_;
    SingleSideDifferentiation<double> y_acceleration_calc_;
    SingleSideDifferentiation<double> z_acceleration_calc_;
protected:
    ros::Subscriber sub_;
    VelocityConverterTwistStamped(boost::function<void (MovementReading)> call_back);
public:
    VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void TwistStampedCB(const geometry_msgs::TwistStamped& msg);
    virtual std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWISTSTAMPED_H
