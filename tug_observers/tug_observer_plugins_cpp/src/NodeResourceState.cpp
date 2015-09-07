//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/NodeResourceState.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.h>

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
      cpu_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", cpu_params),
                                                             cpu_params);

      if (!value.hasMember("memory"))
      {
        ROS_ERROR("No memory for state for node given for resource plugin");
        throw std::runtime_error("No memory for state for node given for resource plugin");
      }
      XmlRpc::XmlRpcValue memory_params = value["memory"];
      memory_ = SingleValueHypothesisCheckFactory<unsigned long>::createSingleValueHypothesisCheck(
              ProcessYaml::getValue<std::string>("type", memory_params), memory_params);

      name_ = ProcessYaml::getValue<std::string>("state", value);
    }

    bool NodeResourceState::conformsState(FilteState<double> cpu_state, FilteState<unsigned long> memory_state)
    {
      ROS_DEBUG_STREAM("conforms state " << name_ << " with cpu: " << cpu_state.value << " mem: " << memory_state.value);
      return cpu_->checkHypothesis(cpu_state) && memory_->checkHypothesis(memory_state);
    }

    std::string NodeResourceState::getName()
    {
      return name_;
    }
}
