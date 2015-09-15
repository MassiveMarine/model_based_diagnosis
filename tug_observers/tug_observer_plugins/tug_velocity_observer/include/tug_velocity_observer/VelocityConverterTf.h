//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTF_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTF_H

#include <tug_velocity_observer/VelocityConverterPoseStamped.h>
#include <nav_msgs/Odometry.h>

class VelocityConverterTf : public VelocityConverterPoseStamped
{
    ros::Rate tf_update_rate_;
    boost::thread background_thread_;
public:
    VelocityConverterTf(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base);
    void run();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTF_H
