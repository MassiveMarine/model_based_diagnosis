//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUEFACTORY_H
#define TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUEFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <XmlRpcValue.h>
#include <tug_observer_plugins_cpp/NominalValue.h>
#include <tug_observer_plugins_cpp/GausNominalValue.h>

template<class T>
class NominalValueFactory
{
public:
  static boost::shared_ptr<NominalValue<T> > createNominalValue(std::string type, XmlRpc::XmlRpcValue params)
  {
    if(type == "gaus")
      return boost::make_shared<GausNominalValue<T> >(params);
    else
      throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
  }

};


#endif //TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUEFACTORY_H
