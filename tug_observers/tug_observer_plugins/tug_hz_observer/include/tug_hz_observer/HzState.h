//
// Created by clemens on 22.09.15.
//

#ifndef TUG_HZ_OBSERVER_HZSTATE_H
#define TUG_HZ_OBSERVER_HZSTATE_H

#include <XmlRpcValue.h>
#include <tug_observer_plugin_utils/filter/FilteState.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <boost/shared_ptr.hpp>

class HzState
{
    std::string name_;
    boost::shared_ptr<SingleValueHypothesisCheck<double> > frequency_;

public:
    HzState(XmlRpc::XmlRpcValue params);
    bool conformsState(FilteState<double> hz_state);
    std::string getName();

};


#endif //TUG_HZ_OBSERVER_HZSTATE_H
