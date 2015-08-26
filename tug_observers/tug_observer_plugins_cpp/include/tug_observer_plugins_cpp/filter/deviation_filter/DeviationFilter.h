//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_DEVIATIONFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_DEVIATIONFILTER_H

#include <stddef.h>
#include <vector>

template<class T>
class DeviationFilter
{
public:
  virtual void update(const T &new_value) = 0;

  virtual std::vector<T> getDeviation() = 0;

  virtual void reset() = 0;

  virtual size_t getSampleSize() = 0;
};


#endif //TUG_OBSERVER_PLUGINS_CPP_DEVIATIONFILTER_H
