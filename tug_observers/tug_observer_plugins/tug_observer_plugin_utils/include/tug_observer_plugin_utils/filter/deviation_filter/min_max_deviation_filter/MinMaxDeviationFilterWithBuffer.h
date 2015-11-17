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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_MIN_MAX_DEVIATION_FILTER_MINMAXDEVIATIONFILTERWITHBUFFER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_MIN_MAX_DEVIATION_FILTER_MINMAXDEVIATIONFILTERWITHBUFFER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <limits>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <tug_yaml/ProcessYaml.h>
#include <vector>

template<class T>
class MinMaxDeviationFilterWithBuffer : public DeviationFilter<T>
{
  boost::circular_buffer<T> buffer_;
  boost::mutex scope_mutex_;

public:
  explicit MinMaxDeviationFilterWithBuffer(XmlRpc::XmlRpcValue params)
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

    for (size_t i = 0; i < buffer_.size(); ++i)
    {
      T current_element = buffer_[i];
      if (current_element < min)
        min = current_element;

      if (current_element > max)
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

#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_MIN_MAX_DEVIATION_FILTER_MINMAXDEVIATIONFILTERWITHBUFFER_H
