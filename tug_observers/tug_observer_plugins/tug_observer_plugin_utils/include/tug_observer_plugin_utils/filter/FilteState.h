//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_FILTESTATE_H
#define TUG_OBSERVER_PLUGINS_CPP_FILTESTATE_H

#include <vector>
#include <stddef.h>
#include <iostream>

template<class T>
struct FilteState
{
  T value;
  std::vector<T> deviation;
  bool has_deviation;
  size_t sample_size;
};

template<class T>
std::ostream& operator<<(std::ostream& out, const FilteState<T>& state){
  out << "value: " << state.value << ", sample size: " << state.sample_size ;

  for(size_t i = 0; i < state.deviation.size(); ++i)
    out << " deviation on position: " << i << " is: " << state.deviation[i];

  return out;
}

#endif //TUG_OBSERVER_PLUGINS_CPP_FILTESTATE_H
