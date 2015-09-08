//
// Created by clemens on 08.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_LINEARINTERPOLATION_H
#define TUG_OBSERVER_PLUGINS_CPP_LINEARINTERPOLATION_H

#include <tug_observer_plugins_cpp/interpolation/Interpolation.h>
#include <list>
#include <stdexcept>

template<class T>
class LinearInterpolation : public Interpolation<T>
{
    std::list<std::pair<T, ros::Time> > a_values_;
    std::list<std::pair<T, ros::Time> > b_values_;

    typename std::list<std::pair<T, ros::Time> >::const_iterator getMaximumLowerBound(
            typename std::list<std::pair<T, ros::Time> >::const_iterator begin,
            typename std::list<std::pair<T, ros::Time> >::const_iterator end, ros::Time interpolation_time)
    {
      typename std::list<std::pair<T, ros::Time> >::const_iterator it;
      for (it = begin; it != end; ++it)
      {
        if (it->second > interpolation_time)
          break;
      }

      return --it;
    }

    T interpolate(const std::list<std::pair<T, ros::Time> > &values, ros::Time interpolation_time)
    {
      typename std::list<std::pair<T, ros::Time> >::const_iterator lower_bound = getMaximumLowerBound(values.begin(),
                                                                                             values.end(),
                                                                                             interpolation_time);
      typename std::list<std::pair<T, ros::Time> >::const_iterator upper_bound = lower_bound;
      upper_bound++;

      if (lower_bound->second == interpolation_time)
        return lower_bound->first;

      if (upper_bound->second == interpolation_time)
        return upper_bound->first;

      double time_difference = (upper_bound->second - lower_bound->second).toSec();
      double time_delay = (interpolation_time - lower_bound->second).toSec();

      T result = lower_bound->first + (upper_bound->first - lower_bound->first) * (time_delay / time_difference);

      return result;
    }

    void clampValues(ros::Time lower_limit, std::list<std::pair<T, ros::Time> > &values)
    {
      typename std::list<std::pair<T, ros::Time> >::const_iterator lower_it = getMaximumLowerBound(values.begin(), values.end(),
                                                                                          lower_limit);

      size_t lower_bound;
      for (typename std::list<std::pair<T, ros::Time> >::const_iterator it = values.begin(), lower_bound = 0;
           it != lower_it; ++it, ++lower_bound);

      if (lower_bound == 0)
        return;

      size_t entries_to_remove = lower_bound - 1;
      for (size_t i = 0; i < entries_to_remove; ++i)
        values.pop_front();
    }

public:
    virtual void addFromA(const T &value, const ros::Time &value_time)
    {
      if (!a_values_.empty() && (value_time < a_values_.back().second))
        throw std::invalid_argument("given time is before last inserted time for a");

      a_values_.push_back(std::make_pair(value, value_time));
    }

    virtual void addFromB(const T &value, const ros::Time &value_time)
    {
      if (!a_values_.empty() && (value_time < a_values_.back().second))
        throw std::invalid_argument("given time is before last inserted time for b");

      b_values_.push_back(std::make_pair(value, value_time));
    }

    virtual bool hasNewInterpolatedPair()
    {
      if (a_values_.empty())
        return false;

      if (b_values_.empty())
        return false;

      ros::Time a_lower_limit = a_values_.front().second;
      ros::Time a_upper_limit = a_values_.back().second;

      ros::Time b_lower_limit = b_values_.front().second;
      ros::Time b_upper_limit = b_values_.back().second;

      if (a_upper_limit < b_lower_limit)
        return false;

      if (b_upper_limit < a_lower_limit)
        return false;

      return true;
    }

    virtual std::pair<T, T> getNextInterpolatedPair()
    {
      if (a_values_.empty())
        throw std::invalid_argument("can't interpolate with empty list of a values");

      if (b_values_.empty())
        throw std::invalid_argument("can't interpolate with empty list of a values");

      ros::Time a_lower_limit = a_values_.front().second;
      ros::Time a_upper_limit = a_values_.back().second;

      ros::Time b_lower_limit = b_values_.front().second;
      ros::Time b_upper_limit = b_values_.back().second;

      // b is in the interval of a
      if ((a_lower_limit <= b_lower_limit) && (b_lower_limit <= a_upper_limit))
      {
        T a_value = interpolate(a_values_, b_lower_limit);
        T b_value = b_values_.front().first;

        clampValues(b_lower_limit, a_values_);
        b_values_.pop_front();
        return std::make_pair(a_value, b_value);
      }// a is in the interval of b
      else if ((b_lower_limit <= a_lower_limit) && (a_lower_limit <= b_upper_limit))
      {
        T a_value = a_values_.front().first;
        T b_value = interpolate(b_values_, a_lower_limit);

        clampValues(a_lower_limit, b_values_);
        a_values_.pop_front();
        return std::make_pair(a_value, b_value);
      }

      throw std::invalid_argument("can't interpolate as intervals do not intersect");
    }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_LINEARINTERPOLATION_H
