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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_EWMAVALUEFILTER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_EWMAVALUEFILTER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <tug_observer_plugin_utils/filter/value_filter/ewma_value_filter/EWMAValueFilterWithBuffer.h>
#include <tug_observer_plugin_utils/filter/value_filter/ewma_value_filter/EWMAValueFilterWithoutBuffer.h>
#include <tug_yaml/ProcessYaml.h>

template<class T>
class EWMAValueFilter : public ValueFilter<T>
{
  boost::shared_ptr<ValueFilter<T> > ewma_internal_value_filter_;

public:
  explicit EWMAValueFilter(XmlRpc::XmlRpcValue params)
  {
    if (ProcessYaml::hasValue("window_size", params))
      ewma_internal_value_filter_ = boost::make_shared<EWMAValueFilterWithBuffer<T> >(params);
    else
      ewma_internal_value_filter_ = boost::make_shared<EWMAValueFilterWithoutBuffer<T> >(params);
  }

  virtual void update(const T& new_value)
  {
    ewma_internal_value_filter_->update(new_value);
  }

  virtual T getValue()
  {
    return ewma_internal_value_filter_->getValue();
  }

  virtual void reset()
  {
    ewma_internal_value_filter_->reset();
  }

  virtual size_t getSampleSize()
  {
    return ewma_internal_value_filter_->getSampleSize();
  }
};

#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_EWMAVALUEFILTER_H
