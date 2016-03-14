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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_NOMINALVALUEHYPOTHESIS_H
#define TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_NOMINALVALUEHYPOTHESIS_H

#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <stdexcept>
#include <tug_yaml/ProcessYaml.h>
#include <tug_observer_plugin_utils/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>
#include <string>
#include <vector>
#include <algorithm>

template <class T>
class NominalValueHypothesis : public SingleValueHypothesisCheck<T>
{
    boost::shared_ptr<NominalValue<T> > value_check_;
    std::vector<boost::shared_ptr<NominalValue<T> > > deviation_check_;
public:
    explicit NominalValueHypothesis(XmlRpc::XmlRpcValue params)
    {
      if (!params.hasMember("value"))
      {
        ROS_ERROR("No value check defined for hypothesis check");
        throw std::runtime_error("No value check defined for hypothesis check");
      }
      XmlRpc::XmlRpcValue value_params = params["value"];
      value_check_ = NominalValueFactory<T>::createNominalValue(ProcessYaml::getValue<std::string>("type",
                                                                                                   value_params),
                                                             value_params);

      if (params.hasMember("deviation"))
      {
        XmlRpc::XmlRpcValue deviations = params["deviation"];
        for (int i = 0; i < deviations.size(); ++i)
          deviation_check_.push_back(NominalValueFactory<T>::createNominalValue(
                  ProcessYaml::getValue<std::string>("type", deviations[i]),
                  deviations[i]));
      }
    }

    virtual bool checkHypothesis(const FilteState<T>& state)
    {
      if (!deviation_check_.empty() && !state.has_deviation)
        throw std::invalid_argument("devition check was created but filter has no devition provided");

      bool result = value_check_->isNominal(state.value);

      if (result && !deviation_check_.empty())
      {
        if (deviation_check_.size() != state.deviation.size())
          throw std::invalid_argument("devition check size does has different size to given deviations");

        for (size_t i = 0; i < deviation_check_.size(); ++i)
        {
          result &= deviation_check_[i]->isNominal(state.deviation[i]);

          if (!result)
            break;
        }
      }

      return result;
    }

    virtual size_t getMinimumSampleSize()
    {
      size_t result = value_check_->getMinimumSampleSize();
      for (size_t i = 0; i < deviation_check_.size(); ++i)
        result = std::max(result, deviation_check_[i]->getMinimumSampleSize());

      return result;
    }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_HYPOTHESIS_CHECK_SINGE_VALUE_HYPOTHESIS_CHECK_NOMINALVALUEHYPOTHESIS_H
