//
// Created by clemens on 28.09.15.
//

#ifndef TUG_OBSERVERS_STATE_H
#define TUG_OBSERVERS_STATE_H

#include <tug_observers_msgs/observation.h>

class Observation
{
  std::string observation_msg_;
  std::string verbose_observation_msg_;
  int32_t observation_code_;

public:
    Observation(std::string observation_msg, int32_t observation_code);
    Observation(std::string observation_msg, std::string verbose_observation_msg, int32_t observation_code);
    tug_observers_msgs::observation toMsg();
};


#endif //TUG_OBSERVERS_STATE_H
