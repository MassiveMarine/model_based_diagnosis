//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_LOWPASSFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_LOWPASSFILTER_H

#include <tug_observer_plugins_cpp/Filter.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>

template<class T>
class LowPassFilter : public Filter<T>
{
  double decay_rate_;
  T current_value_;
  bool got_initial_value_;

public:
  LowPassFilter(XmlRpc::XmlRpcValue params) : got_initial_value_(false)
  {
    decay_rate_ = ProcessYaml::getValue<double>("decay_rate", params);
  }

  virtual void update(const T& new_value)
  {
    if(!got_initial_value_)
    {
      current_value_ = new_value;
      got_initial_value_ = true;
    }
    else
    {
      current_value_ = current_value_ * (1.0 - decay_rate_) + new_value * decay_rate_;
    }
  }

  virtual T getValue()
  {
    if(!got_initial_value_)
      return static_cast<T>(0);

    return current_value_;
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_LOWPASSFILTER_H
