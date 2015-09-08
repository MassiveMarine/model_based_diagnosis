//
// Created by clemens on 26.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTERWITHOUTBUFFER_H
#define TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTERWITHOUTBUFFER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <tug_observer_plugin_utils/ProcessYaml.h>
#include <numeric>

template<class T>
class StandartDeviationFilterWithoutBuffer : public DeviationFilter<T>
{
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
  StandartDeviationFilterWithoutBuffer(XmlRpc::XmlRpcValue params)
  { }

  virtual void update(const T &new_value)
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    history_.push_back(new_value);
  }

  virtual std::vector<T> getDeviation()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    std::vector<T> result;
    result.push_back(summSqaredDifference(history_.begin(), history_.end(), history_.size()));
    return result;
  }

  virtual void reset()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    history_.clear();
  }

  virtual size_t getSampleSize()
  {
    boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    return history_.size();
  }

  virtual size_t getExpectedDeviationResultSize()
  {
    return 1;
  }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_STANDARTDEVIATIONFILTERWITHOUTBUFFER_H
