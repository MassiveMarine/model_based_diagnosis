//
// Created by clemens on 12.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSE_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSE_H

#include <tug_velocity_observer/VelocityConverterPoseStamped.h>
#include <geometry_msgs/Pose.h>

class VelocityConverterPose : public VelocityConverterPoseStamped
{
    std::string topic_;
protected:
    VelocityConverterPose(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back);
public:
    VelocityConverterPose(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void PoseCB(const geometry_msgs::Pose& msg);
    virtual std::string getName();
};

#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSE_H
