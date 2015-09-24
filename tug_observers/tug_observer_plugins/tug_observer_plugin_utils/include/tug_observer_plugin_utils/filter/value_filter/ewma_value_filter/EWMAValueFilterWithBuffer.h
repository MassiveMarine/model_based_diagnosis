//
// Created by clemens on 26.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_EWMAVALUEFILTERWITHBUFFER_H
#define TUG_OBSERVER_PLUGINS_CPP_EWMAVALUEFILTERWITHBUFFER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <tug_yaml/ProcessYaml.h>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

template<class T>
class EWMAValueFilterWithBuffer : public ValueFilter<T>
{
  double decay_rate_;
  boost::mutex scope_mutex_;
  boost::circular_buffer<T> buffer_;

public:
  EWMAValueFilterWithBuffer(XmlRpc::XmlRpcValue params)
  {
    decay_rate_ = ProcessYaml::getValue<double>("decay_rate", params);
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
    T result = static_cast<T>(0);

    for(size_t i = 0; i < buffer_.size(); ++i)
    {
      T current_element = buffer_[i];
      if(i == 0)
      {
        result = current_element;
      }
      else
      {
        result = result * (1.0 - decay_rate_) + current_element * decay_rate_;
      }
    }

    return result;
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



#endif //TUG_OBSERVER_PLUGINS_CPP_EWMAVALUEFILTERWITHBUFFER_H
