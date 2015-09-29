//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

class VelocityConverterPoseStamped : public VelocityConverterTwistStamped
{
    std::string topic_;
    bool has_old_position_;
    geometry_msgs::PoseStamped old_position_;

protected:
    VelocityConverterPoseStamped(boost::function<void (MovementReading)> call_back);
public:
    VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void PoseStampedCB(const geometry_msgs::PoseStamped& msg);
    virtual std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
