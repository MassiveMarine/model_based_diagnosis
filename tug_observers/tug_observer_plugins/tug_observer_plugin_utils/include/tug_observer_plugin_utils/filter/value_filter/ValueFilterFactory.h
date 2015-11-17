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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_VALUEFILTERFACTORY_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_VALUEFILTERFACTORY_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <XmlRpcValue.h>
#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/MeanValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/MedianValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/KMeansValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/EWMAValueFilter.h>
#include <tug_observer_plugin_utils/filter/value_filter/NoValueFilter.h>
#include <string>

template <class T>
class ValueFilterFactory
{
public:
  static boost::shared_ptr<ValueFilter<T> > createFilter(std::string type, XmlRpc::XmlRpcValue params)
  {
    if (type == "mean")
      return boost::make_shared<MeanValueFilter<T> >(params);
    else if (type == "median")
      return boost::make_shared<MedianValueFilter<T> >(params);
    else if (type == "kmeans")
      return boost::make_shared<KMeansValueFilter<T> >(params);
    else if (type == "ewma")
      return boost::make_shared<EWMAValueFilter<T> >(params);
    else if (type == "nofilter")
      return boost::make_shared<NoValueFilter<T> >();
    else
      throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
  }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_VALUEFILTERFACTORY_H
