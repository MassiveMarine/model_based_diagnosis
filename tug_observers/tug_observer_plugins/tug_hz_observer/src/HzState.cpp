//
// Created by clemens on 22.09.15.
//

#include <tug_hz_observer/HzState.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.h>

HzState::HzState(XmlRpc::XmlRpcValue params)
{
  name_ = ProcessYaml::getValue<std::string>("state", params);
  number_ = ProcessYaml::getValue<int32_t>("number", params);
  if (!params.hasMember("frequenzy"))
  {
    ROS_ERROR("No frequenzy for state for a hz state");
    throw std::runtime_error("No frequenzy for state for a hz state");
  }
  XmlRpc::XmlRpcValue rot_x_params = params["frequenzy"];
  frequency_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", rot_x_params), rot_x_params);
}

bool HzState::conformsState(FilteState<double> hz_state)
{
  return frequency_->checkHypothesis(hz_state);
}

std::string HzState::getName()
{
  return  name_;
}

int32_t HzState::getNumber()
{
  return number_;
}