//
// Created by clemens on 07.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_STUDENTSTSINGLEHYPOTHESIS_H
#define TUG_OBSERVER_PLUGINS_CPP_STUDENTSTSINGLEHYPOTHESIS_H



#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValue.h>
#include <stdexcept>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/nominal_value/NominalValueFactory.h>
#include <boost/math/distributions/students_t.hpp>

template <class T>
class StudentsTSingleHypothesis : public SingleValueHypothesisCheck<T>
{
    T true_mean_;
    double significance_level_;

    T absolut(T value)
    {
      if(value < static_cast<T>(0))
        return -value;

      return value;
    }

public:
    StudentsTSingleHypothesis(T true_mean, XmlRpc::XmlRpcValue params) : true_mean_(true_mean)
    {
      significance_level_ = ProcessYaml::getValue<T>("significance_level", params);
    }

    StudentsTSingleHypothesis(XmlRpc::XmlRpcValue params)
    {
      true_mean_ = ProcessYaml::getValue<T>("true_mean", params);
      significance_level_ = ProcessYaml::getValue<T>("significance_level", params);
    }

    virtual bool checkHypothesis(const FilteState<T>& state)
    {
      if(!state.has_deviation || (state.deviation.size() != 1))
        throw std::invalid_argument("student t test needs one deviation as parameter");

      T mean_difference = state.value - true_mean_;
      T t_statistic = mean_difference * std::sqrt(static_cast<T>(state.sample_size)) / state.deviation[0];

      size_t freedoms = state.sample_size - 1;
      boost::math::students_t distribution(freedoms);
      double q = boost::math::cdf(boost::math::complement(distribution, absolut(t_statistic)));

      if(q < significance_level_/2.)
        return false;

      return true;
    }
};

#endif //TUG_OBSERVER_PLUGINS_CPP_STUDENTSTSINGLEHYPOTHESIS_H
