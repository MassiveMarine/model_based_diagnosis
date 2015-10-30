//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUEFACTORY_H
#define TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUEFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <XmlRpcValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/GaussNominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/ExactNominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NotNominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/GreaterThanNominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/LessThanNominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/InBetweenNominalValue.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NotInBetweenNominalValue.h>

template<class T>
class NominalValueFactory
{
public:
  static boost::shared_ptr<NominalValue<T> > createNominalValue(std::string type, XmlRpc::XmlRpcValue params)
  {
    if(type == "gauss")
      return boost::make_shared<GaussNominalValue<T> >(params);
    else if(type == "exact")
      return boost::make_shared<ExactNominalValue<T> >(params);
    else if(type == "not")
      return boost::make_shared<NotNominalValue<T> >(params);
    else if(type == "greather_than")
      return boost::make_shared<GreaterThanNominalValue<T> >(params);
    else if(type == "less_than")
      return boost::make_shared<LessThanNominalValue<T> >(params);
    else if(type == "in_between")
      return boost::make_shared<InBetweenNominalValue<T> >(params);
    else if(type == "not_in_between")
      return boost::make_shared<NotInBetweenNominalValue<T> >(params);
    else
      throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
  }

};


#endif //TUG_OBSERVER_PLUGINS_CPP_NOIMALVALUEFACTORY_H
