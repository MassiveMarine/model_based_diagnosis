//
// Created by clemens on 08.09.15.
//

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYSTATE_H
#define TUG_VELOCITY_OBSERVER_VELOCITYSTATE_H

#include <string>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <boost/shared_ptr.hpp>
#include <XmlRpcValue.h>


class VelocityState
{
    boost::shared_ptr<SingleValueHypothesisCheck<double> > x_;
    boost::shared_ptr<SingleValueHypothesisCheck<double> > y_;
    boost::shared_ptr<SingleValueHypothesisCheck<double> > z_;
    boost::shared_ptr<SingleValueHypothesisCheck<double> > rot_x_;
    boost::shared_ptr<SingleValueHypothesisCheck<double> > rot_y_;
    boost::shared_ptr<SingleValueHypothesisCheck<double> > rot_z_;
    std::string name_;
    int32_t number_;

public:
    VelocityState(XmlRpc::XmlRpcValue value);
    bool conformsStateX(FilteState<double> x_state);
    bool conformsStateY(FilteState<double> y_state);
    bool conformsStateZ(FilteState<double> z_state);
    bool conformsState(FilteState<double> rot_x_state, FilteState<double> rot_y_state, FilteState<double> rot_z_state);
    std::string getName();
    int32_t getNumber();
};


#endif //TUG_VELOCITY_OBSERVER_VELOCITYSTATE_H
