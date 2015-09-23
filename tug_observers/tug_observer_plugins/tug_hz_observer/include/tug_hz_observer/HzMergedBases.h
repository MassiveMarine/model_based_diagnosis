//
// Created by clemens on 22.09.15.
//

#ifndef TUG_HZ_OBSERVER_HZMERGEDBASES_H
#define TUG_HZ_OBSERVER_HZMERGEDBASES_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <tug_hz_observer/HzBase.h>
#include <tug_hz_observer/HzState.h>
#include <tug_observer_plugin_utils/filter/FilteState.h>
#include <tug_observers/ObserverPluginBase.h>

class HzMergedBases
{
    std::vector<boost::shared_ptr<HzBase> > bases_;
    std::vector<HzState> states_;
    tug_observers::ObserverPluginBase* plugin_base_;
    std::string name_;
    ros::Time current_filter_time_;

    FilteState<double> getHzFilterState();
public:
    HzMergedBases(std::string topic, XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base);
    void sendResourceInfo();
    void addBase(boost::shared_ptr<HzBase> base);
};


#endif //TUG_HZ_OBSERVER_HZMERGEDBASES_H
