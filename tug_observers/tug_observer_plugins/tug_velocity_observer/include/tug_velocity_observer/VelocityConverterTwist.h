//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWIST_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWIST_H

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <geometry_msgs/Twist.h>

class VelocityConverterTwist : public VelocityConverterTwistStamped
{
    std::string topic_;
public:
    VelocityConverterTwist(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void TwistCB(const geometry_msgs::Twist& msg);
    virtual std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWIST_H
