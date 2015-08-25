//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NODERESOURCE_H
#define TUG_OBSERVER_PLUGINS_CPP_NODERESOURCE_H

#include <XmlRpcValue.h>
#include <tug_observer_plugins_cpp/NodeResourceState.h>
#include <vector>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilter.h>

namespace tug_observer_plugins_cpp
{
    class NodeResource
    {
    private:
      std::vector<NodeResourceState> states_;
      boost::shared_ptr<ValueFilter<double> > cpu_filter_;
      boost::shared_ptr<ValueFilter<unsigned long> > memory_filter_;

    public:
      NodeResource(XmlRpc::XmlRpcValue &value);

      void update(double cpu, unsigned long memory);

      std::vector<std::string> estimateStates();
    };
}


#endif //TUG_OBSERVER_PLUGINS_CPP_NODERESOURCE_H
