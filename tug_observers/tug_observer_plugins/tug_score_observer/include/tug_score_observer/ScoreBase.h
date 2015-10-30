//
// Created by clemens on 30.10.15.
//

#ifndef TUG_SCORE_OBSERVER_SCOREBASE_H
#define TUG_SCORE_OBSERVER_SCOREBASE_H

#include <tug_observers/ObserverPluginBase.h>
#include <tug_score_observer/ScoreState.h>
#include <tug_observer_plugin_utils/filter/Filter.h>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>

namespace tug_observer_plugins_cpp
{
    class ScoreBase
    {
        /// vector states for the score
        std::vector<ScoreState> states_;

        boost::shared_ptr<Filter<double> > score_filter_;

        ros::Subscriber resource_sub_;

        std::string name_;

        ros::Time last_time_;

        boost::mutex class_mutex_;

        void socresCallback(const std_msgs::Float64::ConstPtr &msg);

    public:
        ScoreBase(std::string name, XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase *plugin_base);

        std::vector<Observation> estimateStates();
        std::string getName();
        ros::Time getLastTime();
    };
}

#endif //TUG_SCORE_OBSERVER_SCOREBASE_H
