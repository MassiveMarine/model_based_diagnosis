//
// Created by clemens on 26.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEANVALUEFILTERWITHOUTBUFFER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEANVALUEFILTERWITHOUTBUFFER_H

#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilter.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <boost/thread/mutex.hpp>

template<class T>
class MeanValueFilterWithoutBuffer : public ValueFilter<T>
{
  T current_value_;
  bool got_initial_value_;
  boost::mutex scope_mutex_;
  size_t sample_size_;

public:
  MeanValueFilterWithoutBuffer(XmlRpc::XmlRpcValue params) : got_initial_value_(false), sample_size_(0)
  { }

  virtual void update(const T& new_value)
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    size_t old_sample_size = sample_size_;
    sample_size_++;
    if(!got_initial_value_)
    {
      current_value_ = new_value;
      got_initial_value_ = true;
    }
    else
    {
      current_value_ = (current_value_ * static_cast<T>(old_sample_size) + new_value)/ static_cast<T>(sample_size_);
    }
  }

  virtual T getValue()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    if(!got_initial_value_)
      return static_cast<T>(0);

    return current_value_;
  }

  virtual void reset()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    got_initial_value_ = false;
    sample_size_ = 0;
  }

  virtual size_t getSampleSize()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    return sample_size_;
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MEANVALUEFILTERWITHOUTBUFFER_H
