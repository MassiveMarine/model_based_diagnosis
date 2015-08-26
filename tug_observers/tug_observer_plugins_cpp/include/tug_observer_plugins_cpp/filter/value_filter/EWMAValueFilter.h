//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_LOWPASSFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_LOWPASSFILTER_H

#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilter.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <tug_observer_plugins_cpp/filter/value_filter/EWMAValueFilter/EWMAValueFilterWithBuffer.h>
#include <tug_observer_plugins_cpp/filter/value_filter/EWMAValueFilter/EWMAValueFilterWithoutBuffer.h>

template<class T>
class EWMAValueFilter : public ValueFilter<T>
{
  boost::shared_ptr<ValueFilter<T> > ewma_internal_value_filter_;

public:
  EWMAValueFilter(XmlRpc::XmlRpcValue params)
  {
    if(ProcessYaml::hasValue("window_size", params))
      ewma_internal_value_filter_ = boost::make_shared<EWMAValueFilterWithBuffer<T> >(params);
    else
      ewma_internal_value_filter_ = boost::make_shared<EWMAValueFilterWithoutBuffer<T> >(params);
  }

  virtual void update(const T& new_value)
  {
    ewma_internal_value_filter_->update(new_value);
  }

  virtual T getValue()
  {
    return ewma_internal_value_filter_->getValue();
  }

  virtual void reset()
  {
    ewma_internal_value_filter_->reset();
  }

  virtual size_t getSampleSize()
  {
    return ewma_internal_value_filter_->getSampleSize();
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_LOWPASSFILTER_H
