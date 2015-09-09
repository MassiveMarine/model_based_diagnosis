//
// Created by clemens on 21.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_LESSTHANNOMINALVALUE_H
#define TUG_OBSERVER_PLUGINS_CPP_LESSTHANNOMINALVALUE_H

#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

template <class T>
class LessThanNominalValue : public NominalValue<T>
{
private:
  T value_;

public:
  LessThanNominalValue(XmlRpc::XmlRpcValue params)
  {
    value_ = ProcessYaml::getValue<T>("less_than", params);
  }

  virtual bool isNominal(const T& value)
  {
    return value < value_;
  }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_LESSTHANNOMINALVALUE_H