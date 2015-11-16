//
// Created by clemens on 28.09.15.
//

#include <tug_observers/Observation.h>

Observation::Observation(std::string observation_msg, int32_t observation_code) : observation_msg_(observation_msg), verbose_observation_msg_(observation_msg), observation_code_(observation_code)
{ }

Observation::Observation(std::string observation_msg, std::string verbose_observation_msg, int32_t observation_code) : observation_msg_(observation_msg), verbose_observation_msg_(verbose_observation_msg), observation_code_(observation_code)
{ }

tug_observers_msgs::observation Observation::toMsg()
{
  tug_observers_msgs::observation result;
  result.observation_msg = observation_msg_;
  result.verbose_observation_msg = verbose_observation_msg_;
  result.observation = observation_code_;

  return result;
}

bool Observation::isFaulty()
{
  return observation_code_ < 0;
}