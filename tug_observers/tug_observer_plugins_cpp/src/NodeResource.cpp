//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/NodeResource.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <ros/ros.h>

namespace tug_observer_plugins_cpp
{

    NodeResource::NodeResource(XmlRpc::XmlRpcValue &value)
    {
      if (!value.hasMember("states"))
      {
        ROS_ERROR("No states for node given for resource plugin");
        throw std::runtime_error("No states for node given for resource plugin");
      }

      XmlRpc::XmlRpcValue params = value["states"];

      for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
        states_.push_back(NodeResourceState(it->second));
    }

    void NodeResource::update(double cpu, unsigned long memory)
    {

    }

    std::vector<std::string> NodeResource::estimateStates()
    {
      return std::vector<std::string>();
    }
}
