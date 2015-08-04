//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NODERESOURCE_H
#define TUG_OBSERVER_PLUGINS_CPP_NODERESOURCE_H

#include <XmlRpcValue.h>
#include <tug_observer_plugins_cpp/NodeResourceState.h>
#include <vector>

namespace tug_observer_plugins_cpp
{
    class NodeResource
    {
    private:
      std::vector<NodeResourceState> states_;

    public:
      NodeResource(XmlRpc::XmlRpcValue &value);

      void update(double cpu, unsigned long memory);

      std::vector<std::string> estimateStates();
    };
}


#endif //TUG_OBSERVER_PLUGINS_CPP_NODERESOURCE_H
