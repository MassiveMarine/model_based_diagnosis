//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_FILTERFACTORY_H
#define TUG_OBSERVER_PLUGINS_CPP_FILTERFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <XmlRpcValue.h>
#include <tug_observer_plugins_cpp/Filter.h>
#include <tug_observer_plugins_cpp/MeanFilter.h>
#include <tug_observer_plugins_cpp/MedianFilter.h>
#include <tug_observer_plugins_cpp/KMeansFilter.h>
#include <tug_observer_plugins_cpp/LowPassFilter.h>
#include <tug_observer_plugins_cpp/NoFilterFilter.h>

template <class T>
class FilterFactory
{
public:
  static boost::shared_ptr<Filter<T> > createFilter(std::string type, XmlRpc::XmlRpcValue params)
  {
    if(type == "mean")
      return boost::make_shared<MeanFilter<T> >(params);
    else if(type == "median")
      return boost::make_shared<MedianFilter<T> >(params);
    else if(type == "kmeans")
      return boost::make_shared<KMeansFilter<T> >(params);
    else if(type == "lp")
      return boost::make_shared<LowPassFilter<T> >(params);
    else if(type == "nofilter")
      return boost::make_shared<NoFilterFilter<T> >();
    else
      throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_FILTERFACTORY_H
