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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_STUDENTSTSINGLEHYPOTHESIS_H
#define TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_STUDENTSTSINGLEHYPOTHESIS_H



#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <stdexcept>
#include <tug_yaml/ProcessYaml.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>
#include <boost/math/distributions/students_t.hpp>

template <class T>
class StudentsTSingleHypothesis : public SingleValueHypothesisCheck<T>
{
    T true_mean_;
    double significance_level_;
    bool has_fixed_deviation_;
    T std_deviation_;

    T absolut(T value)
    {
      if (value < static_cast<T>(0))
        return -value;

      return value;
    }

public:
    StudentsTSingleHypothesis(T true_mean, XmlRpc::XmlRpcValue params) : true_mean_(true_mean)
    {
      significance_level_ = ProcessYaml::getValue<double>("significance_level", params);
      if (ProcessYaml::hasValue("std_deviation", params))
      {
        has_fixed_deviation_ = true;
        std_deviation_ = ProcessYaml::getValue<T>("std_deviation", params);
      }
    }

    explicit StudentsTSingleHypothesis(XmlRpc::XmlRpcValue params)
    {
      true_mean_ = ProcessYaml::getValue<T>("true_mean", params);
      significance_level_ = ProcessYaml::getValue<double>("significance_level", params);
      if (ProcessYaml::hasValue("std_deviation", params))
      {
        has_fixed_deviation_ = true;
        std_deviation_ = ProcessYaml::getValue<T>("std_deviation", params);
      }
    }

    virtual bool checkHypothesis(const FilteState<T>& state)
    {
      T std_deviation;
      if (has_fixed_deviation_)
        std_deviation = std_deviation_;
      else
      {
        if (!state.has_deviation || (state.deviation.size() != 1))
          throw std::invalid_argument("student t test needs one deviation as parameter");

        std_deviation = state.deviation[0];
      }

      T mean_difference = state.value - true_mean_;
      T t_statistic = mean_difference * std::sqrt(static_cast<T>(state.sample_size)) / std_deviation;

      size_t freedoms = state.sample_size - 1;
      ROS_DEBUG_STREAM("check hypothesis with mean: " << mean_difference << " and t_statistic: " << t_statistic
                       << " and freedoms of: " << freedoms);

      boost::math::students_t distribution(freedoms);
      double q = boost::math::cdf(boost::math::complement(distribution, absolut(t_statistic)));

      if (q < significance_level_/2.)
        return false;

      return true;
    }

    virtual size_t getMinimumSampleSize()
    {
      return 2;
    }
};

#endif  // TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_STUDENTSTSINGLEHYPOTHESIS_H
