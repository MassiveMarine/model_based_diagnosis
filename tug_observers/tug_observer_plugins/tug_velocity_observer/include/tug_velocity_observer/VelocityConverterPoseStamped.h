//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

class VelocityConverterPoseStamped : public VelocityConverterTwistStamped
{
    SingleSideDifferentiation<double> linear_x_velocity_calc_;
    SingleSideDifferentiation<double> linear_y_velocity_calc_;
    SingleSideDifferentiation<double> linear_z_velocity_calc_;
    SingleSideDifferentiation<double> angular_x_velocity_calc_;
    SingleSideDifferentiation<double> angular_y_velocity_calc_;
    SingleSideDifferentiation<double> angular_z_velocity_calc_;

protected:
    VelocityConverterPoseStamped(boost::function<void (MovementReading)> call_back);
public:
    VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base);
    void PoseStampedCB(const geometry_msgs::PoseStamped& msg);
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
