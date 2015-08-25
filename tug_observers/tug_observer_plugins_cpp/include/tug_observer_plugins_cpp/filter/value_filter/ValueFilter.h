//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_FILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_FILTER_H

template<class T>
class ValueFilter
{
public:
  virtual void update(const T &new_value) = 0;

  virtual T getValue() = 0;

  virtual void reset() = 0;
};


#endif //TUG_OBSERVER_PLUGINS_CPP_FILTER_H
