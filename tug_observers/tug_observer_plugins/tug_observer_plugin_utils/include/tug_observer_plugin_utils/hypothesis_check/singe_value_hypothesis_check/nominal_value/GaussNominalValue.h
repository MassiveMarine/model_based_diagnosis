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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_NOMINAL_VALUE_GAUSSNOMINALVALUE_H
#define TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_NOMINAL_VALUE_GAUSSNOMINALVALUE_H

#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <tug_yaml/ProcessYaml.h>

template <class T>
class GaussNominalValue : public NominalValue<T>
{
private:
  T mean_;
  T std_deviation_;

  T absolut(T value)
  {
    if (value < static_cast<T>(0))
      return -value;

    return value;
  }

  T distanceToMean(const T& value)
  {
    if (value < mean_)
      return absolut(mean_ - value);

    return absolut(value - mean_);
  }

public:
  explicit GaussNominalValue(XmlRpc::XmlRpcValue params)
  {
    mean_ = ProcessYaml::getValue<T>("mean", params);
    std_deviation_ = ProcessYaml::getValue<T>("std_deviation", params);
  }

  virtual bool isNominal(const T& value)
  {
    T distance = distanceToMean(value);
    ROS_DEBUG_STREAM("got distance to mean " << distance << " allowed is " << std_deviation_);

    if (distance < std_deviation_)
      return true;

    return false;
  }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_NOMINAL_VALUE_GAUSSNOMINALVALUE_H
