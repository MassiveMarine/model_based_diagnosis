//
// Created by clemens on 22.09.15.
//

#ifndef TUG_TIMEOUT_OBSERVER_TIMEOUTBASE_H
#define TUG_TIMEOUT_OBSERVER_TIMEOUTBASE_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <tug_observer_plugin_utils/filter/Filter.h>
#include <tug_observer_plugin_utils/filter/FilteState.h>
#include <XmlRpcValue.h>
#include <tug_observers/ObserverPluginBase.h>

#include <tug_time/Timeout.h>

class TimeoutBase
{
    bool has_last_messurment_;
    ros::Time last_time_;
    double timeout_;
    int max_timeouts_in_a_row_;
    int remaining_timeouts_;
    boost::shared_ptr<Timeout> timeout_thread_;
    std::string name_;
    tug_observers::ObserverPluginBase* plugin_base_;

public:
    TimeoutBase(std::string topic, XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);
    void update();
    bool timeout_callback();
    FilteState<double> getFilterState();
};


#endif //TUG_TIMEOUT_OBSERVER_TIMEOUTBASE_H
