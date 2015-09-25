//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <tug_observer_plugin_utils/filter/deviation_filter/min_max_deviation_filter/MinMaxDeviationFilterWithBuffer.h>
#include <tug_observer_plugin_utils/filter/deviation_filter/min_max_deviation_filter/MinMaxDeviationFilterWithoutBuffer.h>
#include <tug_yaml/ProcessYaml.h>

template<class T>
class MinMaxDeviationFilter : public DeviationFilter<T>
{
  boost::shared_ptr<DeviationFilter<T> > min_max_internal_value_filter_;

public:
  MinMaxDeviationFilter(XmlRpc::XmlRpcValue params)
  {
    if(ProcessYaml::hasValue("window_size", params))
      min_max_internal_value_filter_ = boost::make_shared<MinMaxDeviationFilterWithBuffer<T> >(params);
    else
      min_max_internal_value_filter_ = boost::make_shared<MinMaxDeviationFilterWithoutBuffer<T> >(params);
  }

  virtual void update(const T& new_value)
  {
    min_max_internal_value_filter_->update(new_value);
  }

  virtual std::vector<T> getDeviation()
  {
    return min_max_internal_value_filter_->getDeviation();
  }

  virtual void reset()
  {
    min_max_internal_value_filter_->reset();
  }

  virtual size_t getSampleSize()
  {
    return min_max_internal_value_filter_->getSampleSize();
  }

  virtual size_t getExpectedDeviationResultSize()
  {
    return min_max_internal_value_filter_->getExpectedDeviationResultSize();
  }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTER_H
