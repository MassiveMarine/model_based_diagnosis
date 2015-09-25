//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityState.h>
#include <tug_yaml/ProcessYaml.h>
#include <stdexcept>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.h>

VelocityState::VelocityState(XmlRpc::XmlRpcValue value)
{
  name_ = ProcessYaml::getValue<std::string>("state", value);

  if(value.hasMember("x"))
  {
    XmlRpc::XmlRpcValue x_params = value["x"];
    x_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", x_params), x_params);
  }

  if(value.hasMember("y"))
  {
    XmlRpc::XmlRpcValue y_params = value["y"];
    y_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", y_params), y_params);
  }

  if(value.hasMember("z"))
  {
    XmlRpc::XmlRpcValue z_params = value["z"];
    z_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", z_params), z_params);
  }

  if (!value.hasMember("rot_x"))
  {
    ROS_ERROR("No rotation x for state for a velocity state");
    throw std::runtime_error("No rotation x for state for a velocity state");
  }
  XmlRpc::XmlRpcValue rot_x_params = value["rot_x"];
  rot_x_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", rot_x_params), rot_x_params);

  if (!value.hasMember("rot_y"))
  {
    ROS_ERROR("No rotation y for state for a velocity state");
    throw std::runtime_error("No rotation y for state for a velocity state");
  }
  XmlRpc::XmlRpcValue rot_y_params = value["rot_y"];
  rot_y_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", rot_y_params), rot_y_params);

  if (!value.hasMember("rot_z"))
  {
    ROS_ERROR("No rotation z for state for a velocity state");
    throw std::runtime_error("No rotation z for state for a velocity state");
  }
  XmlRpc::XmlRpcValue rot_z_params = value["rot_z"];
  rot_z_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", rot_z_params), rot_z_params);
}

bool VelocityState::conformsStateX(FilteState<double> x_state)
{
  if(!x_)
    throw std::runtime_error("no hypthesis check for x defined in the state: " + name_);

  return x_->checkHypothesis(x_state);
}

bool VelocityState::conformsStateY(FilteState<double> y_state)
{
  if(!y_)
    throw std::runtime_error("no hypthesis check for y defined in the state: " + name_);

  return y_->checkHypothesis(y_state);
}

bool VelocityState::conformsStateZ(FilteState<double> z_state)
{
  if(!z_)
    throw std::runtime_error("no hypthesis check for z defined in the state: " + name_);

  return z_->checkHypothesis(z_state);
}

bool VelocityState::conformsState(FilteState<double> rot_x_state, FilteState<double> rot_y_state,
                                  FilteState<double> rot_z_state)
{
  return rot_x_->checkHypothesis(rot_x_state) && rot_y_->checkHypothesis(rot_y_state) && rot_z_->checkHypothesis(rot_z_state);
}

std::string VelocityState::getName()
{
  return name_;
}
