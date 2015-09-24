//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEDIANFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEDIANFILTER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <boost/circular_buffer.hpp>
#include <tug_yaml/ProcessYaml.h>
#include <algorithm>
#include <boost/thread/mutex.hpp>

template<class T>
class MedianValueFilter : public ValueFilter<T>
{
  boost::circular_buffer<T> buffer_;
  boost::mutex scope_mutex_;

public:
  MedianValueFilter(XmlRpc::XmlRpcValue params)
  {
    unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
    buffer_ = boost::circular_buffer<T>(window_size);
  }

  virtual void update(const T& new_value)
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    buffer_.push_back(new_value);
  }

  virtual T getValue()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    if(buffer_.empty())
      return static_cast<T>(0);

    size_t upper_median_index = std::ceil((buffer_.size() + 1)/2) - 1;
    size_t lower_median_index = std::floor((buffer_.size() + 1)/2) - 1;

    std::sort(buffer_.begin(), buffer_.end());
    return (buffer_[upper_median_index] + buffer_[lower_median_index]) / static_cast<T>(2);
  }

  virtual void reset()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    buffer_.clear();
  }

  virtual size_t getSampleSize()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    return buffer_.size();
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MEDIANFILTER_H
