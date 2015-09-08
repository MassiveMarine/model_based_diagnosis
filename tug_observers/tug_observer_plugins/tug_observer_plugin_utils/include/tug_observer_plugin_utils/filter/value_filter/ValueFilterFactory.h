//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_FILTERFACTORY_H
#define TUG_OBSERVER_PLUGINS_CPP_FILTERFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <XmlRpcValue.h>
#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/MeanValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/MedianValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/KMeansValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/EWMAValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/NoValueFilter.h>

template <class T>
class ValueFilterFactory
{
public:
  static boost::shared_ptr<ValueFilter<T> > createFilter(std::string type, XmlRpc::XmlRpcValue params)
  {
    if(type == "mean")
      return boost::make_shared<MeanValueFilter<T> >(params);
    else if(type == "median")
      return boost::make_shared<MedianValueFilter<T> >(params);
    else if(type == "kmeans")
      return boost::make_shared<KMeansValueFilter<T> >(params);
    else if(type == "ewma")
      return boost::make_shared<EWMAValueFilter<T> >(params);
    else if(type == "nofilter")
      return boost::make_shared<NoValueFilter<T> >();
    else
      throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_FILTERFACTORY_H
