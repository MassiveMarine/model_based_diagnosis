//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H
#define TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H

#include <XmlRpcValue.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>

namespace tug_observer_plugins_cpp
{
    class ScoreState
    {
      boost::shared_ptr<SingleValueHypothesisCheck<double> > score_;
      std::string name_;
      int32_t number_;

    public:
        ScoreState(XmlRpc::XmlRpcValue value);
        bool conformsState(FilteState<double> score_state);
        std::string getName();
        int32_t getNumber();
    };
}


#endif //TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H
