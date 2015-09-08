//
// Created by clemens on 26.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTERWITHBUFFER_H
#define TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTERWITHBUFFER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <limits>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <tug_observer_plugin_utils/ProcessYaml.h>

template<class T>
class MinMaxDeviationFilterWithBuffer : public DeviationFilter<T>
{
  boost::circular_buffer<T> buffer_;
  boost::mutex scope_mutex_;

public:
  MinMaxDeviationFilterWithBuffer(XmlRpc::XmlRpcValue params)
  {
    unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
    buffer_ = boost::circular_buffer<T>(window_size);
  }

  virtual void update(const T &new_value)
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    buffer_.push_back(new_value);
  }

  virtual std::vector<T> getDeviation()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);

    T min = std::numeric_limits<T>::max();
    T max = std::numeric_limits<T>::min();

    for(size_t i = 0; i < buffer_.size(); ++i)
    {
      T current_element = buffer_[i];
      if(current_element < min)
        min = current_element;

      if(current_element > max)
        max = current_element;
    }

    std::vector<T> result;
    result.push_back(min);
    result.push_back(max);

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

  virtual size_t getExpectedDeviationResultSize()
  {
    return 2;
  }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTERWITHBUFFER_H
