//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/NodeResourceState.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <tug_observer_plugins_cpp/NominalValueFactory.h>

namespace tug_observer_plugins_cpp
{
    NodeResourceState::NodeResourceState(XmlRpc::XmlRpcValue value)
    {
      if (!value.hasMember("cpu"))
      {
        ROS_ERROR("No cpu for state for node given for resource plugin");
        throw std::runtime_error("No cpu for statefor node given for resource plugin");
      }
      XmlRpc::XmlRpcValue cpu_params = value["cpu"];
      cpu_ = NominalValueFactory<double>::createNominalValue(ProcessYaml::getValue<std::string>("type", cpu_params),
                                                             cpu_params);

      if (!value.hasMember("memory"))
      {
        ROS_ERROR("No memory for state for node given for resource plugin");
        throw std::runtime_error("No memory for state for node given for resource plugin");
      }
      XmlRpc::XmlRpcValue memory_params = value["memory"];
      memory_ = NominalValueFactory<unsigned long>::createNominalValue(
              ProcessYaml::getValue<std::string>("type", memory_params), memory_params);

      name_ = ProcessYaml::getValue<std::string>("state", value);
    }

    bool NodeResourceState::conformsState(double cpu, unsigned long memory)
    {
      ROS_ERROR_STREAM("conforms state " << name_ << " with cpu: " << cpu << " mem: " << memory);
      return cpu_->isNominal(cpu) && memory_->isNominal(memory);
    }

    std::string NodeResourceState::getName()
    {
      return name_;
    }
}