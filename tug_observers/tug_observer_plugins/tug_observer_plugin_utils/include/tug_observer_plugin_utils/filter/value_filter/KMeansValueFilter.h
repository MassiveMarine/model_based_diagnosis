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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_KMEANSVALUEFILTER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_KMEANSVALUEFILTER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <boost/circular_buffer.hpp>
#include <tug_yaml/ProcessYaml.h>
#include <numeric>
#include <algorithm>
#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/mutex.hpp>


template<class T>
class KMeansValueFilter : public ValueFilter<T>
{
    boost::circular_buffer<T> buffer_;
    double k_half_;
    boost::mutex scope_mutex_;

    size_t getLowerIndex()
    {
      if (buffer_.size() < 2. * k_half_)
        return 0;

      size_t lower_median_index = std::ceil(static_cast<double>(buffer_.size() + 1) / 2. - k_half_) - 1.;

      return lower_median_index;
    }

    size_t getUpperIndex()
    {
      if (buffer_.size() < 2. * k_half_)
        return buffer_.size() - 1;

      size_t upper_median_index = std::floor(static_cast<double>(buffer_.size() + 1) / 2. + k_half_) - 1.;

      return upper_median_index;
    }

public:
    explicit KMeansValueFilter(XmlRpc::XmlRpcValue params)
    {
      unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
      buffer_ = boost::circular_buffer<T>(window_size);
      if (window_size < 2)
        throw std::invalid_argument("window size must be at least 2 for the k mean value filter");

      k_half_ = static_cast<double>(ProcessYaml::getValue<unsigned int>("k_size", params)) / 2.;
      if (k_half_ < 1.)
        throw std::invalid_argument("k mean must be at least 2 for the k mean value filter");

      if (k_half_ * 2. > window_size)
        throw std::invalid_argument("window size must be at least k size for the k mean value filter");
    }

    virtual void update(const T &new_value)
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      buffer_.push_back(new_value);
    }

    virtual T getValue()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      if (buffer_.empty())
        return static_cast<T>(0);

      ROS_DEBUG_STREAM("get value k means");
      size_t lower_index = getLowerIndex();
      ROS_DEBUG_STREAM("lower index " << lower_index);
      size_t upper_index = getUpperIndex();
      ROS_DEBUG_STREAM("upper index " << upper_index);
      boost::circular_buffer<T> tmp_buffer = buffer_;
      std::sort(tmp_buffer.begin(), tmp_buffer.end());
      ROS_DEBUG("sorted");
      if (upper_index + 1 - lower_index < 2. * k_half_)
        upper_index = std::min(tmp_buffer.size() - 1, upper_index + 1);
      if (upper_index + 1 - lower_index > 2. * k_half_)
        lower_index = std::min(upper_index, lower_index + 1);

      T result = std::accumulate(tmp_buffer.begin() + lower_index, tmp_buffer.begin() + upper_index + 1,
                                 static_cast<T>(0));

      return result / static_cast<T>(upper_index - lower_index + 1);
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
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_KMEANSVALUEFILTER_H
