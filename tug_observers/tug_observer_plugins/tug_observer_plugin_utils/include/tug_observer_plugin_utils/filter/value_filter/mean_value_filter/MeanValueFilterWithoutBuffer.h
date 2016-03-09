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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_MEAN_VALUE_FILTER_MEANVALUEFILTERWITHOUTBUFFER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_MEAN_VALUE_FILTER_MEANVALUEFILTERWITHOUTBUFFER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <tug_yaml/ProcessYaml.h>
#include <boost/thread/mutex.hpp>

template<class T>
class MeanValueFilterWithoutBuffer : public ValueFilter<T>
{
    T current_value_;
    bool got_initial_value_;
    boost::mutex scope_mutex_;
    size_t sample_size_;

public:
    explicit MeanValueFilterWithoutBuffer(XmlRpc::XmlRpcValue params) : got_initial_value_(false), sample_size_(0)
    { }

    virtual void update(const T &new_value)
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      size_t old_sample_size = sample_size_;
      sample_size_++;
      if (!got_initial_value_)
      {
        current_value_ = new_value;
        got_initial_value_ = true;
      }
      else
      {
        current_value_ = (current_value_ * static_cast<T>(old_sample_size) + new_value) / static_cast<T>(sample_size_);
      }
    }

    virtual T getValue()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      if (!got_initial_value_)
        return static_cast<T>(0);

      return current_value_;
    }

    virtual void reset()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      got_initial_value_ = false;
      sample_size_ = 0;
    }

    virtual size_t getSampleSize()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      return sample_size_;
    }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_MEAN_VALUE_FILTER_MEANVALUEFILTERWITHOUTBUFFER_H
