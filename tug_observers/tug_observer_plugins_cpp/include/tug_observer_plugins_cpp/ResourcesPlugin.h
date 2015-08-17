//
// Created by clemens on 04.08.15.
//

#ifndef TUG_OBSERVER_PLUGINS_CPP_RESOURCESPLUGIN_H
#define TUG_OBSERVER_PLUGINS_CPP_RESOURCESPLUGIN_H

#include <tug_observers_cpp/ObserverPluginBase.h>
#include <tug_observer_plugins_cpp/NodeResource.h>
#include <map>
#include <string>
#include <set>
#include <tug_resource_monitor/NodeInfoArray.h>

namespace tug_observer_plugins_cpp
{
    typedef std::vector<tug_resource_monitor::NodeInfo> NodeInfoArray;

    class ResourcesPlugin : public tug_observers_cpp::ObserverPluginBase
    {
    private:
      /// mapping from node names to node resource descriptions
      std::map<std::string, NodeResource> node_resources_;
      /// set to specify all the nodes which should be running
      std::set<std::string> nodes_of_interrest_;

      ros::Subscriber resource_sub_;

      bool received_first_msg_;

      void nodeInfoCallback(const tug_resource_monitor::NodeInfoArray::ConstPtr& msg);

      public:
        ResourcesPlugin();
        virtual void initialize(XmlRpc::XmlRpcValue params);
    };
}

#endif //TUG_OBSERVER_PLUGINS_CPP_RESOURCESPLUGIN_H
