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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_FILTER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_FILTER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugin_utils/filter/FilteState.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_observer_plugin_utils/filter/value_filter/ValueFilterFactory.h>
#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilterFactory.h>
#include <string>

template<class T>
class Filter
{
  boost::shared_ptr<DeviationFilter<T> > deviation_filter_;
  boost::shared_ptr<ValueFilter<T> > value_filter_;

public:
  explicit Filter(XmlRpc::XmlRpcValue params)
  {
    std::string filter_type = ProcessYaml::getValue<std::string>("type", params);
    value_filter_ = ValueFilterFactory<T>::createFilter(filter_type, params);

    if (ProcessYaml::hasValue("deviation_type", params))
    {
      std::string deviation_type = ProcessYaml::getValue<std::string>("deviation_type", params);
      deviation_filter_ = DeviationFilterFactory<T>::createFilter(deviation_type, params);
    }
  }

  void update(const T &new_value)
  {
    if (deviation_filter_)
      deviation_filter_->update(new_value);
    value_filter_->update(new_value);
  }

  FilteState<T> getFilteState()
  {
    FilteState<T> filter_state;
    filter_state.value = value_filter_->getValue();

    if (deviation_filter_)
    {
      filter_state.has_deviation = true;
      filter_state.deviation = deviation_filter_->getDeviation();
    }
    else
      filter_state.has_deviation = false;

    filter_state.sample_size = value_filter_->getSampleSize();

    return filter_state;
  }

  void reset()
  {
    if (deviation_filter_)
      deviation_filter_->reset();
    value_filter_->reset();
  }

  size_t getExpectedDeviationResultSize()
  {
    if (deviation_filter_)
      return deviation_filter_->getExpectedDeviationResultSize();

    return 0;
  }

  bool hasDeviationFilter()
  {
    return deviation_filter_;
  }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_FILTER_H
