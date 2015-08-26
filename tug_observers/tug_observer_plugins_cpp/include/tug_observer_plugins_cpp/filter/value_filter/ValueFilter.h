//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_VALUE_FILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_VALUE_FILTER_H

#include <stddef.h>

template<class T>
class ValueFilter
{
public:
  virtual void update(const T &new_value) = 0;

  virtual T getValue() = 0;

  virtual void reset() = 0;

  virtual size_t getSampleSize() = 0;
};


#endif //TUG_OBSERVER_PLUGINS_CPP_VALUE_FILTER_H
