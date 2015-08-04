//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/ResourcesPlugin.h>
#include <ros/ros.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>

namespace tug_observer_plugins_cpp
{
    void ResourcesPlugin::initialize(const ros::NodeHandle &nh)
    {
      XmlRpc::XmlRpcValue params;
      nh.getParam("nodes", params);

      if (!params.valid())
      {
        ROS_ERROR("No nodes given for resource plugin");
        throw std::runtime_error("No nodes given for resource plugin");
      }

      for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
      {
        XmlRpc::XmlRpcValue &param = it->second;

        std::string name = ProcessYaml::getValue<std::string>("name", param);
        nodes_of_interrest_.insert(name);

        NodeResource new_resource(param);

        node_resources_.insert(std::make_pair(name, new_resource));
      }
    }

    void ResourcesPlugin::nodeInfoCallback(const tug_resource_monitor::NodeInfoArray::ConstPtr &msg)
    {
      std::set<std::string> remaining_nodes = nodes_of_interrest_;
      std::vector<std::map<std::string, NodeResource>::iterator> found_node_resources;
      for (NodeInfoArray::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
      {
        switch (it->error)
        {
          case tug_resource_monitor::NodeInfo::NO_ERROR:
            {
              std::map<std::string, NodeResource>::iterator node_it = node_resources_.find(it->name);
              if (node_it == node_resources_.end())
                continue;

              node_it->second.update(it->cpu, it->memory);

              remaining_nodes.erase(it->name);
              found_node_resources.push_back(node_it);
            }
            break;
          case tug_resource_monitor::NodeInfo::ERROR_PID_NOT_FOUND: // nothing to do as remaining nodes will have this node in its set
            break;
        }
      }

      for(std::set<std::string>::iterator it = remaining_nodes.begin(); it != remaining_nodes.end(); ++it)
        reportError(*it, "not_running_" + *it, "The node with the name '" + *it + "' is not running");

      for(std::vector<std::map<std::string, NodeResource>::iterator>::iterator it = found_node_resources.begin(); it != found_node_resources.end(); ++it)
      {
        std::string name = (*it)->first;
        std::vector<std::string> states = (*it)->second.estimateStates();
        reportStates(name, states);
      }
    }

}


PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ResourcesPlugin, tug_observers_cpp::ObserverPluginBase)