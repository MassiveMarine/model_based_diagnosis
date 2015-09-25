//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERODOMETRY_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERODOMETRY_H


#include <tug_velocity_observer/VelocityConverterPoseStamped.h>
#include <nav_msgs/Odometry.h>

class VelocityConverterOdometry : public VelocityConverterPoseStamped
{
    std::string topic_;
public:
    VelocityConverterOdometry(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void OdometryCB(const nav_msgs::Odometry& msg);
    virtual std::string getName();
};

#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERODOMETRY_H
