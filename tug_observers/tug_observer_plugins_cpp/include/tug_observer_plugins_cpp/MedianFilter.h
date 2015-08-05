//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MEDIANFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MEDIANFILTER_H

#include <tug_observer_plugins_cpp/Filter.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <algorithm>

template<class T>
class MedianFilter : public Filter<T>
{
  boost::circular_buffer<T> buffer_;

public:
  MedianFilter(XmlRpc::XmlRpcValue params)
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

    size_t upper_median_index = std::ceil((buffer_.size() + 1)/2) - 1;
    size_t lower_median_index = std::floor((buffer_.size() + 1)/2) - 1;

    std::sort(buffer_.begin(), buffer_.end());
    return (buffer_[upper_median_index] + buffer_[lower_median_index]) / static_cast<T>(2);
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MEDIANFILTER_H
