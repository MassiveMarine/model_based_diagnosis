//
// Created by clemens on 21.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_EXACTNOMINALVALUE_H
#define TUG_OBSERVER_PLUGINS_CPP_EXACTNOMINALVALUE_H

#include <tug_observer_plugins_cpp/NominalValue.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>

template <class T>
class ExactNominalValue : public NominalValue<T>
{
private:
  T value_;

public:
  ExactNominalValue(XmlRpc::XmlRpcValue params)
  {
    value_ = ProcessYaml::getValue<T>("exact", params);
  }

  virtual bool isNominal(const T& value)
  {
    return value == value_;
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_EXACTNOMINALVALUE_H
