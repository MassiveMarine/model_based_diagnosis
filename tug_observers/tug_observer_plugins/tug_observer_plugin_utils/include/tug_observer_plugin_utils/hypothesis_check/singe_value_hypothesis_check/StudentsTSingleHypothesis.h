//
// Created by clemens on 07.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_STUDENTSTSINGLEHYPOTHESIS_H
#define TUG_OBSERVER_PLUGINS_CPP_STUDENTSTSINGLEHYPOTHESIS_H



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
      if(value < static_cast<T>(0))
        return -value;

      return value;
    }

public:
    StudentsTSingleHypothesis(T true_mean, XmlRpc::XmlRpcValue params) : true_mean_(true_mean)
    {
      significance_level_ = ProcessYaml::getValue<double>("significance_level", params);
      if(ProcessYaml::hasValue("std_deviation", params))
      {
        has_fixed_deviation_ = true;
        std_deviation_ = ProcessYaml::getValue<T>("std_deviation", params);
      }
    }

    StudentsTSingleHypothesis(XmlRpc::XmlRpcValue params)
    {
      true_mean_ = ProcessYaml::getValue<T>("true_mean", params);
      significance_level_ = ProcessYaml::getValue<double>("significance_level", params);
      if(ProcessYaml::hasValue("std_deviation", params))
      {
        has_fixed_deviation_ = true;
        std_deviation_ = ProcessYaml::getValue<T>("std_deviation", params);
      }
    }

    virtual bool checkHypothesis(const FilteState<T>& state)
    {
      T std_deviation;
      if(has_fixed_deviation_)
        std_deviation = std_deviation_;
      else
      {
        if(!state.has_deviation || (state.deviation.size() != 1))
          throw std::invalid_argument("student t test needs one deviation as parameter");

        std_deviation = state.deviation[0];
      }

      T mean_difference = state.value - true_mean_;
      T t_statistic = mean_difference * std::sqrt(static_cast<T>(state.sample_size)) / std_deviation;

      size_t freedoms = state.sample_size - 1;
      ROS_DEBUG_STREAM("check hypothesis with mean: " << mean_difference << " and t_statistic: " << t_statistic << " and freedoms of: " << freedoms);

      boost::math::students_t distribution(freedoms);
      double q = boost::math::cdf(boost::math::complement(distribution, absolut(t_statistic)));

      if(q < significance_level_/2.)
        return false;

      return true;
    }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_STUDENTSTSINGLEHYPOTHESIS_H
