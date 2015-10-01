//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWISTSTAMPED_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWISTSTAMPED_H

#include <tug_velocity_observer/VelocityConverter.h>
#include <tug_observers/SubscriberFacade.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tug_observer_plugin_utils/differentiation/Differentiation.h>

class VelocityConverterTwistStamped : public VelocityConverter
{
    std::string topic_;
    boost::shared_ptr<Differentiation<double> > x_acceleration_calc_;
    boost::shared_ptr<Differentiation<double> > y_acceleration_calc_;
    boost::shared_ptr<Differentiation<double> > z_acceleration_calc_;
protected:
    ros::Publisher twist_pub_;
    ros::Publisher movement_pub_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back);
public:
    VelocityConverterTwistStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void TwistStampedCB(const geometry_msgs::TwistStamped& msg);
    virtual std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERTWISTSTAMPED_H
