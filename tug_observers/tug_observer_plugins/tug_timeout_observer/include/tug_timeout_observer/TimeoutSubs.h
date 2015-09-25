//
// Created by clemens on 22.09.15.
//

#ifndef TUG_TIMEOUT_OBSERVER_TIMEOUTSUBS_H
#define TUG_TIMEOUT_OBSERVER_TIMEOUTSUBS_H

#include <XmlRpcValue.h>
#include <tug_observers/ObserverPluginBase.h>
#include <map>
#include <string>
#include <tug_timeout_observer/TimeoutBase.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <set>

class TimeoutSubs
{
    std::string topic_;
    ros::Subscriber sub_;
    boost::mutex bases_lock_;
    std::map<std::string, boost::shared_ptr<TimeoutBase> > callerids_config_;
//    std::set<boost::shared_ptr<TimeoutMergedBases> > merged_bases_;
    boost::shared_ptr<XmlRpc::XmlRpcValue> default_config_;
//    boost::shared_ptr<TimeoutMergedBases> default_merged_bases_;
    tug_observers::ObserverPluginBase* plugin_base_;

public:
    TimeoutSubs(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);
    void cb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event);
//    void sendResourceInfo();
};


#endif //TUG_TIMEOUT_OBSERVER_TIMEOUTSUBS_H
