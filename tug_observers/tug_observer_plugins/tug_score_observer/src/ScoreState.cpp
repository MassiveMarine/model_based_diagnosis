//
// Created by clemens on 04.08.15.
//

#include <tug_score_observer/ScoreState.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheckFactory.h>

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
      score_ = SingleValueHypothesisCheckFactory<double>::createSingleValueHypothesisCheck(ProcessYaml::getValue<std::string>("type", score_params),
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
}
