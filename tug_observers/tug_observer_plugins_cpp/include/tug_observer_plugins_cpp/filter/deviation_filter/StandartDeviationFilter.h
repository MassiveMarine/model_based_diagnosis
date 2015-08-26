//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTER_H

#include <tug_observer_plugins_cpp/filter/deviation_filter/DeviationFilter.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <tug_observer_plugins_cpp/filter/deviation_filter/standart_deviation_filter/StandartDeviationFilterWithBuffer.h>
#include <tug_observer_plugins_cpp/filter/deviation_filter/standart_deviation_filter/StandartDeviationFilterWithoutBuffer.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>

template<class T>
class StandartDeviationFilter : public DeviationFilter<T>
{
  boost::shared_ptr<DeviationFilter<T> > std_internal_value_filter_;

public:
  StandartDeviationFilter(XmlRpc::XmlRpcValue params)
  {
    if(ProcessYaml::hasValue("window_size", params))
      std_internal_value_filter_ = boost::make_shared<StandartDeviationFilterWithBuffer<T> >(params);
    else
      std_internal_value_filter_ = boost::make_shared<StandartDeviationFilterWithoutBuffer<T> >(params);
  }

  virtual void update(const T& new_value)
  {
    std_internal_value_filter_->update(new_value);
  }

  virtual std::vector<T> getDeviation()
  {
    return std_internal_value_filter_->getDeviation();
  }

  virtual void reset()
  {
    std_internal_value_filter_->reset();
  }

  virtual size_t getSampleSize()
  {
    return std_internal_value_filter_->getSampleSize();
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTER_H
