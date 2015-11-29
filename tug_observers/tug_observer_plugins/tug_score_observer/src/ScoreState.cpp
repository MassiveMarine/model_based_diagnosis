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

#include <tug_score_observer/ScoreState.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.h>
#include <string>

namespace tug_observer_plugins_cpp
{
    ScoreState::ScoreState(XmlRpc::XmlRpcValue value)
    {
      if (!value.hasMember("score"))
      {
        ROS_ERROR("No score for state for node given for scores plugin");
        throw std::runtime_error("No score for statefor node given for scores plugin");
      }
      XmlRpc::XmlRpcValue score_params = value["score"];
      score_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(
              ProcessYaml::getValue<std::string>("type", score_params),
                                                             score_params);

      name_ = ProcessYaml::getValue<std::string>("state", value);
      number_ = ProcessYaml::getValue<int32_t>("number", value);
    }

    bool ScoreState::conformsState(FilteState<double> score_state)
    {
      ROS_DEBUG_STREAM("conforms state " << name_ << " with score: " << score_state.value);
      return score_->checkHypothesis(score_state);
    }

    std::string ScoreState::getName()
    {
      return name_;
    }

    int32_t ScoreState::getNumber()
    {
      return number_;
    }
}  // namespace tug_observer_plugins_cpp
