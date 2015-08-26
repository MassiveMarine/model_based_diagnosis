//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_DEVIATIONFILTERFACTORY_H
#define TUG_OBSERVER_PLUGINS_CPP_DEVIATIONFILTERFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <XmlRpcValue.h>
#include <tug_observer_plugins_cpp/filter/deviation_filter/DeviationFilter.h>
#include <tug_observer_plugins_cpp/filter/deviation_filter/MinMaxDeviationFilter.h>
#include "StandartDeviationFilter.h"

template <class T>
class DeviationFilterFactory
{
public:
  static boost::shared_ptr<DeviationFilter<T> > createFilter(std::string type, XmlRpc::XmlRpcValue params)
  {
    if(type == "min_max")
      return boost::make_shared<MinMaxDeviationFilter<T> >(params);
    else if(type == "std")
      return boost::make_shared<StandartDeviationFilter<T> >(params);
    else
      throw std::runtime_error("type for deviation filter value '" + type + "'" + " not known");
  }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_DEVIATIONFILTERFACTORY_H
