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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_INTERPOLATION_LINEARINTERPOLATION_H
#define TUG_OBSERVER_PLUGIN_UTILS_INTERPOLATION_LINEARINTERPOLATION_H

#include <tug_observer_plugin_utils/interpolation/Interpolation.h>
#include <list>
#include <stdexcept>
#include <utility>
#include <ros/ros.h>

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
      typename std::list<std::pair<T, ros::Time> >::const_iterator lower_it = getMaximumLowerBound(values.begin(),
                                                                                                   values.end(),
                                                                                          lower_limit);

      size_t lower_bound = 0;
      for (typename std::list<std::pair<T, ros::Time> >::const_iterator it = values.begin();
           it != lower_it; ++it)
      {
        ++lower_bound;
      }

      if (lower_bound == 0)
        return;

      size_t entries_to_remove = lower_bound - 1;
      for (size_t i = 0; i < entries_to_remove; ++i)
        values.pop_front();
    }

public:
    virtual void addFromA(const T &value, const ros::Time &value_time)
    {
      ROS_DEBUG("add from a called");
      if (!a_values_.empty() && (value_time < a_values_.back().second))
      {
        ROS_DEBUG_STREAM("want to insert value " << value << " with timing: " << value_time.toSec() <<
                                 " but last timing is " << a_values_.back().second.toSec() <<
                                 " of elements" << a_values_.size());
        throw std::invalid_argument("given time is before last inserted time for a");
      }

      a_values_.push_back(std::make_pair(value, value_time));
    }

    virtual void addFromB(const T &value, const ros::Time &value_time)
    {
      if (!b_values_.empty() && (value_time < b_values_.back().second))
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
      }  // a is in the interval of b
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


#endif  // TUG_OBSERVER_PLUGIN_UTILS_INTERPOLATION_LINEARINTERPOLATION_H
