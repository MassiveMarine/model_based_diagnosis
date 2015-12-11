/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <tug_moveing_observer/VelocityState.h>
#include <tug_yaml/ProcessYaml.h>
#include <stdexcept>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.h>
#include <string>

VelocityState::VelocityState(XmlRpc::XmlRpcValue value)
{
  name_ = ProcessYaml::getValue<std::string>("state", value);
  number_ = ProcessYaml::getValue<int32_t>("number", value);

  if (value.hasMember("x"))
  {
    XmlRpc::XmlRpcValue x_params = value["x"];
    x_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
            ProcessYaml::getValue<std::string>("type", x_params), x_params);
  }

  if (value.hasMember("y"))
  {
    XmlRpc::XmlRpcValue y_params = value["y"];
    y_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
            ProcessYaml::getValue<std::string>("type", y_params), y_params);
  }

  if (value.hasMember("z"))
  {
    XmlRpc::XmlRpcValue z_params = value["z"];
    z_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
            ProcessYaml::getValue<std::string>("type", z_params), z_params);
  }

  if (!value.hasMember("rot_x"))
  {
    ROS_ERROR("No rotation x for state for a velocity state");
    throw std::runtime_error("No rotation x for state for a velocity state");
  }
  XmlRpc::XmlRpcValue rot_x_params = value["rot_x"];
  rot_x_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
          ProcessYaml::getValue<std::string>("type", rot_x_params), rot_x_params);

  if (!value.hasMember("rot_y"))
  {
    ROS_ERROR("No rotation y for state for a velocity state");
    throw std::runtime_error("No rotation y for state for a velocity state");
  }
  XmlRpc::XmlRpcValue rot_y_params = value["rot_y"];
  rot_y_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
          ProcessYaml::getValue<std::string>("type", rot_y_params), rot_y_params);

  if (!value.hasMember("rot_z"))
  {
    ROS_ERROR("No rotation z for state for a velocity state");
    throw std::runtime_error("No rotation z for state for a velocity state");
  }
  XmlRpc::XmlRpcValue rot_z_params = value["rot_z"];
  rot_z_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
          ProcessYaml::getValue<std::string>("type", rot_z_params), rot_z_params);
}

bool VelocityState::conformsStateX(FilteState<double> x_state)
{
  if (!x_)
    throw std::runtime_error("no hypthesis check for x defined in the state: " + name_);

  return x_->checkHypothesis(x_state);
}

bool VelocityState::conformsStateY(FilteState<double> y_state)
{
  if (!y_)
    throw std::runtime_error("no hypthesis check for y defined in the state: " + name_);

  return y_->checkHypothesis(y_state);
}

bool VelocityState::conformsStateZ(FilteState<double> z_state)
{
  if (!z_)
    throw std::runtime_error("no hypthesis check for z defined in the state: " + name_);

  return z_->checkHypothesis(z_state);
}

bool VelocityState::conformsState(FilteState<double> rot_x_state, FilteState<double> rot_y_state,
                                  FilteState<double> rot_z_state)
{
  return rot_x_->checkHypothesis(rot_x_state) && rot_y_->checkHypothesis(rot_y_state) &&
         rot_z_->checkHypothesis(rot_z_state);
}

std::string VelocityState::getName()
{
  return name_;
}

int32_t VelocityState::getNumber()
{
  return number_;
}

bool VelocityState::canCheckX(FilteState<double> x_state)
{
  if (!x_)
    throw std::runtime_error("no hypthesis check for x defined in the state: " + name_);

  return x_->getMinimumSampleSize() <= x_state.sample_size;
}

bool VelocityState::canCheckY(FilteState<double> y_state)
{
  if (!y_)
    throw std::runtime_error("no hypthesis check for y defined in the state: " + name_);

  return y_->getMinimumSampleSize() <= y_state.sample_size;
}

bool VelocityState::canCheckZ(FilteState<double> z_state)
{
  if (!z_)
    throw std::runtime_error("no hypthesis check for z defined in the state: " + name_);

  return z_->getMinimumSampleSize() <= z_state.sample_size;
}

bool VelocityState::canCheck(FilteState<double> rot_x_state, FilteState<double> rot_y_state,
                             FilteState<double> rot_z_state)
{
  if (rot_x_->getMinimumSampleSize() > rot_x_state.sample_size)
    return false;
  if (rot_y_->getMinimumSampleSize() > rot_y_state.sample_size)
    return false;
  if (rot_z_->getMinimumSampleSize() > rot_z_state.sample_size)
    return false;
}
