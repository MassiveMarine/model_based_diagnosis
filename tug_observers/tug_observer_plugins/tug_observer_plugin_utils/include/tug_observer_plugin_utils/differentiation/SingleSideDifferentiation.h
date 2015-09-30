//
// Created by clemens on 12.09.15.
//

#ifndef TUG_OBSERVER_PLUGIN_UTILS_SINGLESIDEDIFFERENTIATION_H
#define TUG_OBSERVER_PLUGIN_UTILS_SINGLESIDEDIFFERENTIATION_H

#include <tug_observer_plugin_utils/differentiation/Differentiation.h>
#include <ros/ros.h>

template <class T>
class SingleSideDifferentiation : public Differentiation<T>
{
    T past_value_;
    ros::Time past_value_time_;
    bool has_past_value_;
    T current_differntiation_;
    bool has_current_differntiation_;
    ros::Time current_differntiation_time_;

public:
    SingleSideDifferentiation() : has_past_value_(false), has_current_differntiation_(false)
    { }

    virtual void addValue(const T &value, const ros::Time& value_time)
    {
      if(has_past_value_)
      {
        if(value_time == past_value_time_)
        {
          ROS_DEBUG_STREAM("skip value for calculation single side differntiation with new value:" << value <<
                          " and old value:" << past_value_ << " current time sec:" << value_time.sec <<
                          " nsec:" << value_time.nsec << " old time sec: " << past_value_time_.sec <<
                          " nsec:" << past_value_time_.nsec);
          return;
        } else if(value_time < past_value_time_)
        {
          ROS_ERROR_STREAM("can't calculate single side differntiation with new value:" << value <<
                           " and old value:" << past_value_ << " current time sec:" << value_time.sec <<
                           " nsec:" << value_time.nsec << " old time sec: " << past_value_time_.sec <<
                           " nsec:" << past_value_time_.nsec);
          throw std::invalid_argument("new added value is in the past can't process this data");
        }
        ROS_DEBUG_STREAM("calculate single side differntiation with new value:" << value <<
                                 " and old value:" << past_value_ << " current time sec:" << value_time.sec <<
                                " nsec:" << value_time.nsec << " old time sec: " << past_value_time_.sec <<
                                " nsec:" << past_value_time_.nsec);
        double time_difference = (value_time - past_value_time_).toSec();
        ROS_DEBUG_STREAM("time difference:" << time_difference);
        ROS_DEBUG_STREAM("value difference: " << static_cast<double>(value - past_value_));
        current_differntiation_ = static_cast<T>(static_cast<double>(value - past_value_) / time_difference);
        current_differntiation_time_ = value_time;
        has_current_differntiation_ = true;
        ROS_DEBUG_STREAM("calculated difference:" << current_differntiation_ << " at time:" << current_differntiation_time_);
      }

      past_value_time_ = value_time;
      past_value_ = value;
      has_past_value_ = true;
    }

    virtual bool hasDifferentiation()
    {
      return  has_current_differntiation_;
    }

    virtual T getDifferentiation()
    {
      return  current_differntiation_;
    }

    virtual ros::Time getDifferntiationTime()
    {
      return current_differntiation_time_;
    }
};


#endif //TUG_OBSERVER_PLUGIN_UTILS_SINGLESIDEDIFFERENTIATION_H
