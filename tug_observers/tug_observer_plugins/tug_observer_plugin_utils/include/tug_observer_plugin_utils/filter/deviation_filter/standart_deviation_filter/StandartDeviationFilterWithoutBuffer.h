/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_STANDART_DEVIATION_FILTER_STANDARTDEVIATIONFILTERWITHOUTBUFFER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_STANDART_DEVIATION_FILTER_STANDARTDEVIATIONFILTERWITHOUTBUFFER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <tug_yaml/ProcessYaml.h>
#include <numeric>
#include <vector>

template<class T>
class StandartDeviationFilterWithoutBuffer : public DeviationFilter<T>
{
  std::vector<T> history_;
  boost::mutex scope_mutex_;

  template<typename Iterator>
  T summSqaredDifference(Iterator begin, Iterator end, T mean_value)
  {
    T result = static_cast<T>(0);
    for (Iterator it = begin; it != end; ++it)
    {
      T tmp_result = (*it) - mean_value;
      result += tmp_result * tmp_result;
    }

    return result;
  }

  template<typename Iterator>
  T getMeanValue(Iterator begin, Iterator end, size_t size)
  {
    if (size == 0)
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
  explicit StandartDeviationFilterWithoutBuffer(XmlRpc::XmlRpcValue params)
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

#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_STANDART_DEVIATION_FILTER_STANDARTDEVIATIONFILTERWITHOUTBUFFER_H
