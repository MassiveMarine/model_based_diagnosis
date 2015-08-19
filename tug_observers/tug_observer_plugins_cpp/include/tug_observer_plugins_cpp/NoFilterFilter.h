//
// Created by clemens on 19.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_NOFILTERFILTER_H
#define TUG_OBSERVER_PLUGINS_CPP_NOFILTERFILTER_H

#include <tug_observer_plugins_cpp/Filter.h>
#include <boost/thread/mutex.hpp>

template<class T>
class NoFilterFilter : public Filter<T>
{
    T current_value_;
    boost::mutex scope_mutex_;

    public:
    NoFilterFilter()
    {
      current_value_ = static_cast<T>(0);
    }

    virtual void update(const T& new_value)
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
    }

    virtual T getValue()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      return current_value_;
    }

    virtual void reset()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      current_value_ = static_cast<T>(0);
    }
};


#endif //TUG_OBSERVER_PLUGINS_CPP_NOFILTERFILTER_H
