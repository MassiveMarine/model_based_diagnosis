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
    class NodeResourceState
    {
      boost::shared_ptr<SingleValueHypothesisCheck<double> > cpu_;
      boost::shared_ptr<SingleValueHypothesisCheck<unsigned long> > memory_;
      std::string name_;

    public:
        NodeResourceState(XmlRpc::XmlRpcValue value);
        bool conformsState(FilteState<double> cpu_state, FilteState<unsigned long> memory_state);
        std::string getName();
    };
}


#endif //TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H
