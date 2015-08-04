//
// Created by clemens on 03.08.15.
//

#ifndef TUG_OBSERVERS_CPP_OBSERVERNODE_H
#define TUG_OBSERVERS_CPP_OBSERVERNODE_H

#include <tug_plugin_manager/plugin_manager.h>
#include <tug_observers_cpp/ObserverPluginBase.h>
#include <ros/ros.h>

namespace tug_observers_cpp
{

    typedef tug_plugin_manager::PluginManager<ObserverPluginBase> PluginManager;
    typedef boost::shared_ptr<ObserverPluginBase> ObserverPluginBasePtr;
    typedef std::vector<tug_plugin_manager::PluginSpec<ObserverPluginBase> > Observers;

    class ObserverNode
    {
    private:
      PluginManager plugin_manager_;
      ros::NodeHandle nh_;

    public:
      ObserverNode(ros::NodeHandle nh);

      void initPlugins();

      void startPlugins();
    };

}


#endif //TUG_OBSERVERS_CPP_OBSERVERNODE_H
