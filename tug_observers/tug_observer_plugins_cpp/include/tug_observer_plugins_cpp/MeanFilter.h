//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H

#include <tug_observer_plugins_cpp/Filter.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <numeric>
#include <boost/thread/mutex.hpp>


template<class T>
class MeanFilter : public Filter<T>
{
  boost::circular_buffer<T> buffer_;
  boost::mutex scope_mutex_;

public:
  MeanFilter(XmlRpc::XmlRpcValue params)
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
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H