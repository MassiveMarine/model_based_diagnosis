//
// Created by clemens on 21.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NOTINBETWEENNOMINALVALUE_H
#define TUG_OBSERVER_PLUGINS_CPP_NOTINBETWEENNOMINALVALUE_H


#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

template <class T>
class NotInBetweenNominalValue : public NominalValue<T>
{
private:
  T lower_bound_;
  T upper_bound_;

public:
  NotInBetweenNominalValue(XmlRpc::XmlRpcValue params)
  {
    lower_bound_ = ProcessYaml::getValue<T>("lower_bound", params);
    upper_bound_ = ProcessYaml::getValue<T>("upper_bound", params);

    if(lower_bound_ > upper_bound_)
      throw std::runtime_error("lower bound is greater than upper bound for not in between");
  }

  virtual bool isNominal(const T& value)
  {
    return (value < lower_bound_) || (upper_bound_ < value);
  }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_NOTINBETWEENNOMINALVALUE_H
