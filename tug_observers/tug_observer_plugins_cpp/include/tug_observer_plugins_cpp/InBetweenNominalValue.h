//
// Created by clemens on 21.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_INBETWEENNOMINALVALUE_H
#define TUG_OBSERVER_PLUGINS_CPP_INBETWEENNOMINALVALUE_H

#include <tug_observer_plugins_cpp/NominalValue.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>

template <class T>
class InBetweenNominalValue : public NominalValue<T>
{
private:
  T lower_bound_;
  T upper_bound_;

public:
  InBetweenNominalValue(XmlRpc::XmlRpcValue params)
  {
    lower_bound_ = ProcessYaml::getValue<T>("lower_bound", params);
    upper_bound_ = ProcessYaml::getValue<T>("upper_bound", params);

    if(lower_bound_ > upper_bound_)
      throw std::runtime_error("lower bound is greater than upper bound for not in between");
  }

  virtual bool isNominal(const T& value)
  {
    return (lower_bound_ < value) && (value < upper_bound_);
  }
};



#endif //TUG_OBSERVER_PLUGINS_CPP_INBETWEENNOMINALVALUE_H
