//
// Created by clemens on 12.09.15.
//

#ifndef TUG_OBSERVER_PLUGIN_UTILS_TWOSIDEDIFFERENTIATION_H
#define TUG_OBSERVER_PLUGIN_UTILS_TWOSIDEDIFFERENTIATION_H

#include <tug_observer_plugin_utils/differentiation/Differentiation.h>

template <class T>
class TwoSideDifferentiation : public Differentiation<T>
{
    T past1_value_;
    ros::Time past1_value_time_;
    bool has_past1_value_;

    T past2_value_;
    ros::Time past2_value_time_;
    bool has_past2_value_;

    T current_differntiation_;
    bool has_current_differntiation_;
    ros::Time current_differntiation_time_;

public:
    TwoSideDifferentiation() : has_past1_value_(false), has_past2_value_(false), has_current_differntiation_(false)
    { }

    virtual void addValue(const T &value, const ros::Time& value_time)
    {
      if(has_past2_value_)
      {
        if(value_time <= past2_value_time_)
          throw std::invalid_argument("new added value is in the past can't process this data");

        if(has_past1_value_)
        {
          double time_difference = (value_time - past1_value_time_).toSec();
          current_differntiation_ = static_cast<T>(static_cast<double>(past1_value_ - value) / time_difference);
          current_differntiation_time_ = past2_value_time_;
          has_current_differntiation_ = true;
        }

        past1_value_time_ = past2_value_time_;
        past1_value_ = past2_value_;
        has_past1_value_ = true;
      }

      past2_value_time_ = value_time;
      past2_value_ = value;
      has_past2_value_ = true;
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


#endif //TUG_OBSERVER_PLUGIN_UTILS_TWOSIDEDIFFERENTIATION_H
