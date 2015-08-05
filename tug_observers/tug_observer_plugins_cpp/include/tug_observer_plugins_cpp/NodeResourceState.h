//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H
#define TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H

#include <XmlRpcValue.h>
#include <tug_observer_plugins_cpp/NominalValue.h>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace tug_observer_plugins_cpp
{
    class NodeResourceState
    {
      boost::shared_ptr<NominalValue<double> > cpu_;
      boost::shared_ptr<NominalValue<unsigned long> > memory_;
      std::string name_;

    public:
        NodeResourceState(XmlRpc::XmlRpcValue value);
        bool conformsState(double cpu, unsigned long memory);
        std::string getName();
    };
}


#endif //TUG_OBSERVER_PLUGINS_CPP_NODERESOURCESTATE_H
