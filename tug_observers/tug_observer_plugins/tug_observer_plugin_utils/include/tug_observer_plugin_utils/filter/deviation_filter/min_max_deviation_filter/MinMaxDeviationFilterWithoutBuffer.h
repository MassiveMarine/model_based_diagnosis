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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_MIN_MAX_DEVIATION_FILTER_MINMAXDEVIATIONFILTERWITHOUTBUFFER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_MIN_MAX_DEVIATION_FILTER_MINMAXDEVIATIONFILTERWITHOUTBUFFER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <limits>
#include <tug_yaml/ProcessYaml.h>
#include <vector>

template<class T>
class MinMaxDeviationFilterWithoutBuffer : public DeviationFilter<T>
{
  T min_;
  T max_;
  size_t sample_size_;

public:
  explicit MinMaxDeviationFilterWithoutBuffer(XmlRpc::XmlRpcValue params)
  {
    min_ = std::numeric_limits<T>::max();
    max_ = std::numeric_limits<T>::min();
    sample_size_ = 0;
  }

  virtual void update(const T &new_value)
  {
    if (new_value < min_)
      min_ = new_value;

    if (new_value > max_)
      max_ = new_value;

    sample_size_++;
  }

  virtual std::vector<T> getDeviation()
  {
    std::vector<T> result;
    result.push_back(min_);
    result.push_back(max_);
  }

  virtual void reset()
  {
    min_ = std::numeric_limits<T>::max();
    max_ = std::numeric_limits<T>::min();
    sample_size_ = 0;
  }

  virtual size_t getSampleSize()
  {
    return sample_size_;
  }

  virtual size_t getExpectedDeviationResultSize()
  {
    return 2;
  }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_DEVIATION_FILTER_MIN_MAX_DEVIATION_FILTER_MINMAXDEVIATIONFILTERWITHOUTBUFFER_H
