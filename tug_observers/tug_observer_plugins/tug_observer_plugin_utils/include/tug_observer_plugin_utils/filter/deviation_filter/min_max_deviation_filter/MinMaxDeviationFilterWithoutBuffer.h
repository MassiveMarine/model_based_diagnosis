//
// Created by clemens on 26.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTERWITHOUTBUFFER_H
#define TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTERWITHOUTBUFFER_H

#include <tug_observer_plugin_utils/filter/deviation_filter/DeviationFilter.h>
#include <limits>
#include <tug_yaml/ProcessYaml.h>


template<class T>
class MinMaxDeviationFilterWithoutBuffer : public DeviationFilter<T>
{
  T min_;
  T max_;
  size_t sample_size_;

public:
  MinMaxDeviationFilterWithoutBuffer(XmlRpc::XmlRpcValue params)
  {
    min_ = std::numeric_limits<T>::max();
    max_ = std::numeric_limits<T>::min();
    sample_size_ = 0;
  }

  virtual void update(const T &new_value)
  {
    if(new_value < min_)
      min_ = new_value;

    if(new_value > max_)
      max_ = new_value;

    sample_size_++;
  }

  virtual std::vector<T> getDeviation()
  {
    std::vector<T> result;
    result.push_back(min_);
    result.push_back(max_);
  }

  virtual void reset()
  {
    min_ = std::numeric_limits<T>::max();
    max_ = std::numeric_limits<T>::min();
    sample_size_ = 0;
  }

  virtual size_t getSampleSize()
  {
    return sample_size_;
  }

  virtual size_t getExpectedDeviationResultSize()
  {
    return 2;
  }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTERWITHOUTBUFFER_H
