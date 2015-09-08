//
// Created by clemens on 08.09.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_INTERPOLATION_H
#define TUG_OBSERVER_PLUGINS_CPP_INTERPOLATION_H

#include <utility>
#include <ros/time.h>

template<class T>
class Interpolation
{
public:
    virtual void addFromA(const T &value, const ros::Time& value_time) = 0;
    virtual void addFromB(const T &value, const ros::Time& value_time) = 0;
    virtual bool hasNewInterpolatedPair() = 0;
    virtual std::pair<T, T> getNextInterpolatedPair() = 0;
};

#endif //TUG_OBSERVER_PLUGINS_CPP_INTERPOLATION_H
