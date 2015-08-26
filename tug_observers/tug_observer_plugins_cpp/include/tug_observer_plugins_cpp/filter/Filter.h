//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_FILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_FILTER_H

#include <tug_observer_plugins_cpp/filter/deviation_filter/DeviationFilter.h>
#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilter.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugins_cpp/filter/FilteState.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilterFactory.h>
#include <tug_observer_plugins_cpp/filter/deviation_filter/DeviationFilterFactory.h>


template<class T>
class Filter
{
  boost::shared_ptr<DeviationFilter<T> > deviation_filter_;
  boost::shared_ptr<ValueFilter<T> > value_filter_;

public:
  Filter(XmlRpc::XmlRpcValue params)
  {
    std::string filter_type = ProcessYaml::getValue<std::string>("type", params);
    value_filter_ = ValueFilterFactory<T>::createFilter(filter_type, params);

    if(ProcessYaml::hasValue("deviation_type"))
    {
      std::string deviation_type = ProcessYaml::getValue<std::string>("deviation_type", params);
      deviation_filter_ = DeviationFilterFactory<T>::createFilter(deviation_type, params);
    }
  }

  void update(const T &new_value)
  {
    deviation_filter_->update(new_value);
    value_filter_->update(new_value);
  }

  FilteState<T> getFilteState()
  {
    FilteState<T> filter_state;
    filter_state.value = value_filter_->getValue();

    if(deviation_filter_)
    {
      filter_state.has_deviation = true;
      filter_state.deviation = deviation_filter_->getDeviation();
    }
    else
      filter_state.has_deviation = false;

    filter_state.sample_size = value_filter_->getSampleSize();
  }

  void reset()
  {
    deviation_filter_->reset();
    value_filter_->reset();
  }

};


#endif //TUG_OBSERVER_PLUGINS_CPP_FILTER_H
