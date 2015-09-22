//
// Created by clemens on 22.09.15.
//

#ifndef TUG_HZ_OBSERVER_HZBASE_H
#define TUG_HZ_OBSERVER_HZBASE_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <tug_observer_plugin_utils/filter/Filter.h>
#include <tug_observer_plugin_utils/filter/FilteState.h>
#include <XmlRpcValue.h>

class HzBase
{
    bool has_last_messurment_;
    ros::Time last_time_;
    boost::shared_ptr<Filter<double> > time_filter_;

public:
    HzBase(XmlRpc::XmlRpcValue params);
    void update();
    FilteState<double> getFilterState();
};


#endif //TUG_HZ_OBSERVER_HZBASE_H
