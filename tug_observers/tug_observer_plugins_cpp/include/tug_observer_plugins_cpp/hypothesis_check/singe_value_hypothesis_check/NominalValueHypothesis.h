//
// Created by clemens on 07.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NOMINALVALUEHYPOTHESIS_H
#define TUG_OBSERVER_PLUGINS_CPP_NOMINALVALUEHYPOTHESIS_H

#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <stdexcept>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>

template <class T>
class NominalValueHypothesis : public SingleValueHypothesisCheck<T>
{

    boost::shared_ptr<NominalValue<T> > value_check_;
    std::vector<boost::shared_ptr<NominalValue<T> > > deviation_check_;
public:
    NominalValueHypothesis(XmlRpc::XmlRpcValue params)
    {
      if (!params.hasMember("value"))
      {
        ROS_ERROR("No value check defined for hypothesis check");
        throw std::runtime_error("No value check defined for hypothesis check");
      }
      XmlRpc::XmlRpcValue value_params = params["value"];
      value_check_ = NominalValueFactory<T>::createNominalValue(ProcessYaml::getValue<std::string>("type", value_params),
                                                             value_params);

      if(params.hasMember("deviation"))
      {
        XmlRpc::XmlRpcValue deviations = params["deviation"];
        for(int i = 0; i < deviations.size(); ++i)
          deviation_check_.push_back(NominalValueFactory<T>::createNominalValue(ProcessYaml::getValue<std::string>("type", deviations[i]),
                                                                                deviations[i]));
      }
    }

    virtual bool checkHypothesis(const FilteState<T>& state)
    {
      if(!deviation_check_.empty() && !state.has_deviation)
        throw std::invalid_argument("devition check was created but filter has no devition provided");

      bool result = value_check_->isNominal(state.value);

      if(result && !deviation_check_.empty())
      {
        if(deviation_check_.size() != state.deviation.size())
          throw std::invalid_argument("devition check size does has different size to given deviations");

        for(size_t i = 0; i < deviation_check_.size(); ++i)
        {
          result &= deviation_check_[i]->isNominal(state.deviation[i]);

          if(!result)
            break;
        }
      }

      return result;
    }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_NOMINALVALUEHYPOTHESIS_H
