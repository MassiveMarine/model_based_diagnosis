//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H

#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilter.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <tug_observer_plugins_cpp/filter/value_filter/MeanValueFilter/MeanValueFilterWithBuffer.h>
#include <tug_observer_plugins_cpp/filter/value_filter/MeanValueFilter/MeanValueFilterWithoutBuffer.h>


template<class T>
class MeanValueFilter : public ValueFilter<T>
{
  boost::shared_ptr<ValueFilter<T> > mean_internal_value_filter_;

public:
  MeanValueFilter(XmlRpc::XmlRpcValue params)
  {
    if(ProcessYaml::hasValue("window_size", params))
      mean_internal_value_filter_ = boost::make_shared<MeanValueFilterWithBuffer<T> >(params);
    else
      mean_internal_value_filter_ = boost::make_shared<MeanValueFilterWithoutBuffer<T> >(params);
  }

  virtual void update(const T& new_value)
  {
    mean_internal_value_filter_->update(new_value);
  }

  virtual T getValue()
  {
    return mean_internal_value_filter_->getValue();
  }

  virtual void reset()
  {
    mean_internal_value_filter_->reset();
  }

  virtual size_t getSampleSize()
  {
    return mean_internal_value_filter_->getSampleSize();
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H
