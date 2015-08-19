//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_KMEANSFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_KMEANSFILTER_H

#include <tug_observer_plugins_cpp/Filter.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <numeric>
#include <algorithm>
#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/mutex.hpp>


template<class T>
class KMeansFilter : public Filter<T>
{
  boost::circular_buffer<T> buffer_;
  unsigned int k_half_;
  boost::mutex scope_mutex_;

  size_t getLowerIndex()
  {
    size_t lower_median_index = std::floor((buffer_.size() + 1)/2) - 1;
    if(k_half_ > lower_median_index)
      return 0;

    return lower_median_index - k_half_;
  }

  size_t getUpperIndex()
  {
    size_t upper_median_index = std::ceil((buffer_.size() + 1)/2) - 1;
    size_t result = upper_median_index + k_half_;
    if(result < buffer_.size())
      return result;

    return  buffer_.size() - 1;
  }

public:
  KMeansFilter(XmlRpc::XmlRpcValue params)
  {
    unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
    buffer_ = boost::circular_buffer<T>(window_size);
    k_half_ = ProcessYaml::getValue<unsigned int>("k_size", params) / 2;
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

    ROS_DEBUG_STREAM("get value k means");
    size_t upper_index = getUpperIndex();
    ROS_DEBUG_STREAM("upper index " << upper_index);
    size_t lower_index = getLowerIndex();
    ROS_DEBUG_STREAM("lower index " << lower_index);
    std::sort(buffer_.begin(), buffer_.end());
    ROS_DEBUG("sorted");
    T result = std::accumulate(buffer_.begin() + lower_index, buffer_.begin() + upper_index, static_cast<T>(0));

    return result / static_cast<T>(upper_index - lower_index);
  }

  virtual void reset()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    buffer_.clear();
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_KMEANSFILTER_H
