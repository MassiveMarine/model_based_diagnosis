//
// Created by clemens on 07.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_SINGLEVALUEHYPOTHESISCHECKFACTORY_H
#define TUG_OBSERVER_PLUGINS_CPP_SINGLEVALUEHYPOTHESISCHECKFACTORY_H

#include <boost/smart_ptr/shared_ptr.hpp>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/SingleValueHypothesisCheck.h>
#include <XmlRpcValue.h>
#include <stdexcept>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/NominalValueHypothesis.h>
#include <tug_observer_plugins_cpp/hypothesis_check/singe_value_hypothesis_check/StudentsTSingleHypothesis.h>

template<class T>
class SingleValueHypothesisCheckFactory
{
public:
    static boost::shared_ptr<SingleValueHypothesisCheck<T> > createSingleValueHypothesisCheck(std::string type, XmlRpc::XmlRpcValue params)
    {
      if(type == "nominal_value")
        return boost::make_shared<NominalValueHypothesis<T> >(params);
      else if(type == "student_t")
        return boost::make_shared<StudentsTSingleHypothesis<T> >(params);
      else
        throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
    }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_SINGLEVALUEHYPOTHESISCHECKFACTORY_H
