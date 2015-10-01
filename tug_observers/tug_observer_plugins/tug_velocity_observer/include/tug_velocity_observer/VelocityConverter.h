//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTER_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTER_H

#include <tug_velocity_observer/MovementReading.h>
#include <boost/function.hpp>
#include <tug_observer_plugin_utils/filter/Filter.h>

class VelocityConverter
{
    boost::function<void (MovementReading)> call_back_;
    boost::shared_ptr<Filter<double> > x_filter_;
    boost::shared_ptr<Filter<double> > y_filter_;
    boost::shared_ptr<Filter<double> > z_filter_;
    boost::shared_ptr<Filter<double> > rot_x_filter_;
    boost::shared_ptr<Filter<double> > rot_y_filter_;
    boost::shared_ptr<Filter<double> > rot_z_filter_;
protected:
    VelocityConverter(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back);
    void sendMovement(MovementReading twist);
public:
    virtual std::string getName() = 0;
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYCONVERTER_H
