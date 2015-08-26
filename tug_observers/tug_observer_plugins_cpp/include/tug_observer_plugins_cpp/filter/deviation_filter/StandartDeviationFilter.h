//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTER_H

#include <tug_observer_plugins_cpp/filter/deviation_filter/DeviationFilter.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <numeric>

template<class T>
class StandartDeviationFilter : public DeviationFilter<T>
{
  bool has_buffer_;
  boost::circular_buffer<T> buffer_;
  std::vector<T> history_;
  boost::mutex scope_mutex_;

  template<typename Iterator>
  T summSqaredDifference(Iterator begin, Iterator end, T mean_value)
  {
    T result = static_cast<T>(0);
    for(Iterator it = begin; it != end; ++it)
    {
      T tmp_result = (*it) - mean_value;
      result += tmp_result * tmp_result;
    }

    return result;
  }

  template<typename Iterator>
  T getMeanValue(Iterator begin, Iterator end, size_t size)
  {
    if(size)
      return static_cast<T>(0);

    T result = std::accumulate(begin, end, static_cast<T>(0));
    return result / static_cast<T>(size);
  }

  template<typename Iterator>
  T calculateStd(Iterator begin, Iterator end, size_t size)
  {
    T mean_value = getMeanValue(begin, end, size);
    T sum_differences = summSqaredDifference(begin, end, mean_value);

    return std::sqrt<T>(sum_differences / (static_cast<T>(size) - static_cast<T>(1)));
  }

public:
  StandartDeviationFilter(XmlRpc::XmlRpcValue params)
  {
    if(ProcessYaml::hasValue("window_size"))
    {
      unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
      buffer_ = boost::circular_buffer<T>(window_size);
      has_buffer_ = true;
    }
    else
      has_buffer_ = false;
  }

  virtual void update(const T &new_value)
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);

    if(has_buffer_)
      buffer_.push_back(new_value);
    else
      history_.push_back(new_value);
  }

  virtual std::vector<T> getDeviation()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);

    std::vector<T> result;

    if(has_buffer_)
      result.push_back(summSqaredDifference(buffer_.begin(), buffer_.end(), buffer_.size()));
    else
      result.push_back(summSqaredDifference(history_.begin(), history_.end(), history_.size()));

    return result;
  }

  virtual void reset()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    buffer_.clear();
    history_.clear();
  }

  virtual size_t getSampleSize()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);

    if(has_buffer_)
      return buffer_.size();
    else
      return history_.size();
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTER_H
