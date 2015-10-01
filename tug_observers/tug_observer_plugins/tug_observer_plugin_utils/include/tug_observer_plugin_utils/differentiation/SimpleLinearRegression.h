//
// Created by clemens on 01.10.15.
//

#ifndef TUG_OBSERVER_PLUGIN_UTILS_SIMPLELINEARREGRESSION_H
#define TUG_OBSERVER_PLUGIN_UTILS_SIMPLELINEARREGRESSION_H

#include <tug_observer_plugin_utils/differentiation/Differentiation.h>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <numeric>

/**
 * we use beta of the simple linear regression as derivative
 */
template <class T>
class SimpleLinearRegression : public Differentiation<T>
{
    /*
     * for the implementation we use x as the time
     * y is the value which should be derived
     */
    boost::circular_buffer<T> value_buffer_;
    boost::circular_buffer<ros::Time> time_buffer_;
    boost::mutex scope_mutex_;
    unsigned int window_size_;

    ros::Time calculateMeanTime(const boost::circular_buffer<ros::Time>& buffer)
    {
      if(buffer.empty())
        return ros::Time(0);

      ros::Time upper_time = buffer.back();
      ros::Time lower_time = buffer.front();
      ros::Duration time_diff = upper_time - lower_time;

      return lower_time + (time_diff * (1./2.));
    }

    template <class S>
    S calculateMean(const boost::circular_buffer<S>& buffer)
    {
      if(buffer.empty())
        return static_cast<S>(0);

      S result = std::accumulate(buffer.begin(), buffer.end(), static_cast<S>(0));
      return result / static_cast<S>(buffer.size());
    }

public:
    SimpleLinearRegression(unsigned int window_size) : window_size_(window_size)
    {
      value_buffer_ = boost::circular_buffer<T>(window_size);
      time_buffer_ = boost::circular_buffer<ros::Time>(window_size);
    }

    virtual void addValue(const T &value, const ros::Time& value_time)
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      value_buffer_.push_back(value);
      time_buffer_.push_back(value_time);
    }

    virtual bool hasDifferentiation()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      return value_buffer_.size() == window_size_;
    }

    virtual T getDifferentiation()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      T mean_value = calculateMean(value_buffer_);
      ros::Time mean_time = calculateMeanTime(time_buffer_);

      T numerator = static_cast<T>(0);
      T denominator = 0;
      for(size_t i = 0; i < value_buffer_.size(); ++i)
      {
        T time_diff = static_cast<T>((time_buffer_[i] - mean_time).toSec());
        numerator += time_diff * (value_buffer_[i] - mean_value);
        denominator += time_diff * time_diff;
      }

      return numerator / denominator;
    }

    virtual ros::Time getDifferntiationTime()
    {
      ros::Time tmp_time;
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      return calculateMeanTime(time_buffer_);
    }

    void clear()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      value_buffer_.clear();
      time_buffer_.clear();
    }
};


#endif //TUG_OBSERVER_PLUGIN_UTILS_SIMPLELINEARREGRESSION_H
