//
// Created by clemens on 26.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEANVALUEFILTERWITHBUFFER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEANVALUEFILTERWITHBUFFER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugin_utils/ProcessYaml.h>
#include <numeric>
#include <boost/thread/mutex.hpp>


template<class T>
class MeanValueFilterWithBuffer : public ValueFilter<T>
{
  boost::circular_buffer<T> buffer_;
  boost::mutex scope_mutex_;

public:
  MeanValueFilterWithBuffer(XmlRpc::XmlRpcValue params)
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

    T result = std::accumulate(buffer_.begin(), buffer_.end(), static_cast<T>(0));
    return result / static_cast<T>(buffer_.size());
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

#endif //TUG_OBSERVER_PLUGINS_CPP_MEANVALUEFILTERWITHBUFFER_H
