//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_GAUSNOMINALVALUE_H
#define TUG_OBSERVER_PLUGINS_CPP_GAUSNOMINALVALUE_H

#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>

template <class T>
class GaussNominalValue : public NominalValue<T>
{
private:
  T mean_;
  T std_deviation_;

  T absolut(T value)
  {
    if(value < static_cast<T>(0))
      return -value;

    return value;
  }

  T distanceToMean(const T& value)
  {
    if(value < mean_)
      return absolut(mean_ - value);

    return absolut(value - mean_);
  }

public:
  GaussNominalValue(XmlRpc::XmlRpcValue params)
  {
    mean_ = ProcessYaml::getValue<T>("mean", params);
    std_deviation_ = ProcessYaml::getValue<T>("std_deviation", params);
  }

  virtual bool isNominal(const T& value)
  {
    T distance = distanceToMean(value);
    ROS_DEBUG_STREAM("got distance to mean " << distance << " allowed is " << std_deviation_);

    if(distance < std_deviation_)
      return true;

    return false;
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_GAUSNOMINALVALUE_H
