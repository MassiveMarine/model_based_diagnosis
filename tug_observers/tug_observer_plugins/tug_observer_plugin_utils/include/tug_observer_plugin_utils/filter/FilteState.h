//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_FILTESTATE_H
#define TUG_OBSERVER_PLUGINS_CPP_FILTESTATE_H

#include <vector>
#include <stddef.h>

template<class T>
struct FilteState
{
  T value;
  std::vector<T> deviation;
  bool has_deviation;
  size_t sample_size;
};


#endif //TUG_OBSERVER_PLUGINS_CPP_FILTESTATE_H
