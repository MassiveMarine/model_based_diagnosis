//
// Created by clemens on 22.09.15.
//

#ifndef TUG_HZ_OBSERVER_HZSUBS_H
#define TUG_HZ_OBSERVER_HZSUBS_H

#include <XmlRpcValue.h>
#include <tug_observers/ObserverPluginBase.h>
#include <map>
#include <string>
#include <tug_hz_observer/HzBase.h>
#include <boost/thread/mutex.hpp>
#include <tug_hz_observer/HzMergedBases.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <set>

class HzSubs
{
    std::string topic_;
    ros::Subscriber sub_;
    boost::mutex bases_lock_;
    std::map<std::string, boost::shared_ptr<HzBase> > callerids_config_;
    std::set<boost::shared_ptr<HzMergedBases> > merged_bases_;
    boost::shared_ptr<XmlRpc::XmlRpcValue> default_config_;
    boost::shared_ptr<HzMergedBases> default_merged_bases_;

public:
    HzSubs(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);
    void cb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event);
    void sendResourceInfo();
};


#endif //TUG_HZ_OBSERVER_HZSUBS_H
