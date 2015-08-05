//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H

#include <tug_observer_plugins_cpp/Filter.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <numeric>

template<class T>
class MeanFilter : public Filter<T>
{
  boost::circular_buffer<T> buffer_;

public:
  MeanFilter(XmlRpc::XmlRpcValue params)
  {
    unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
    buffer_ = boost::circular_buffer<T>(window_size);
  }

  virtual void update(const T& new_value)
  {
    buffer_.push_back(new_value);
  }

  virtual T getValue()
  {
    if(buffer_.empty())
      return static_cast<T>(0);

    T result = std::accumulate(buffer_.begin(), buffer_.end(), static_cast<T>(0));
    return result / static_cast<T>(buffer_.size());
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MEANFILTER_H
