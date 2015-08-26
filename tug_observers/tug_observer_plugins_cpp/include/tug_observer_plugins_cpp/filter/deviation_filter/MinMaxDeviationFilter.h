//
// Created by clemens on 25.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTER_H


#include <tug_observer_plugins_cpp/filter/deviation_filter/DeviationFilter.h>
#include <limits>

template<class T>
class MinMaxDeviationFilter : public DeviationFilter<T>
{
  T min_;
  T max_;
  size_t sample_size_;

public:
  MinMaxDeviationFilter()
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
};


#endif //TUG_OBSERVER_PLUGINS_CPP_MINMAXDEVIATIONFILTER_H
