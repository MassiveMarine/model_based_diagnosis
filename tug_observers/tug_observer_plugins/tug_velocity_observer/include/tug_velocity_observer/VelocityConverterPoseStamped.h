//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugin_utils/differentiation/SimpleLinearRegression.h>

class VelocityConverterPoseStamped : public VelocityConverterTwistStamped
{
    std::string topic_;
    unsigned int window_size_;
    boost::circular_buffer<geometry_msgs::PoseStamped> pose_buffer_;
    boost::shared_ptr<SimpleLinearRegression<double> > linear_x_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > linear_y_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > linear_z_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > angular_x_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > angular_y_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > angular_z_velocity_calc_;

protected:
    VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params,boost::function<void (MovementReading)> call_back);
public:
    VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base);
    void PoseStampedCB(const geometry_msgs::PoseStamped& msg);
    virtual std::string getName();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
