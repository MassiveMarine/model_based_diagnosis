//
// Created by clemens on 12.09.15.
//

#ifndef TUG_OBSERVER_PLUGIN_UTILS_DIFFERENTIATION_H
#define TUG_OBSERVER_PLUGIN_UTILS_DIFFERENTIATION_H

#include <ros/time.h>

template <class T>
class Differentiation
{
public:
    virtual void addValue(const T &value, const ros::Time& value_time) = 0;
    virtual bool hasDifferentiation() = 0;
    virtual T getDifferentiation() = 0;
    virtual ros::Time getDifferntiationTime() = 0;
};


#endif //TUG_OBSERVER_PLUGIN_UTILS_DIFFERENTIATION_H
