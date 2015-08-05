//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/ResourcesPlugin.h>
#include <ros/ros.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>

namespace tug_observer_plugins_cpp
{

    void ResourcesPlugin::initialize(XmlRpc::XmlRpcValue params)
    {
      ROS_DEBUG("[ResourcesPlugin::initialize] 1");
      if (!params.hasMember("nodes"))
      {
        ROS_DEBUG("No nodes given for resource plugin");
        throw std::runtime_error("No nodes given for resource plugin");
      }
      ROS_DEBUG("[ResourcesPlugin::initialize] 2");
      XmlRpc::XmlRpcValue nodes = params["nodes"];
      for (int i = 0; i < nodes.size(); ++i)
      {
        ROS_DEBUG("[ResourcesPlugin::initialize] 2.1");
        XmlRpc::XmlRpcValue &param = nodes[i];
        ROS_DEBUG("[ResourcesPlugin::initialize] 2.2");
        std::string name = ProcessYaml::getValue<std::string>("name", param);
        nodes_of_interrest_.insert(name);
        ROS_DEBUG("[ResourcesPlugin::initialize] 2.3");
        NodeResource new_resource(param);
        ROS_DEBUG("[ResourcesPlugin::initialize] 2.4");
        node_resources_.insert(std::make_pair(name, new_resource));
      }
      ROS_DEBUG("[ResourcesPlugin::initialize] 3");
      received_first_msg_ = false;

      resource_sub_ = subscribe("/robot_41/diag/node_infos", 1, &ResourcesPlugin::nodeInfoCallback, this);
    }

    void ResourcesPlugin::nodeInfoCallback(const tug_resource_monitor::NodeInfoArray::ConstPtr &msg)
    {
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 1");
      std::set<std::string> remaining_nodes = nodes_of_interrest_;
      std::vector<std::map<std::string, NodeResource>::iterator> found_node_resources;
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 2");
      for (NodeInfoArray::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
      {
        ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 2.1 [" << it->name << "]");
        switch (it->error)
        {
          case tug_resource_monitor::NodeInfo::NO_ERROR:
            {
              ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 2.1.1 [" << it->name << "]");
              std::map<std::string, NodeResource>::iterator node_it = node_resources_.find(it->name);
              if (node_it == node_resources_.end())
                continue;

              ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 2.1.2 [" << it->name << "]");
              node_it->second.update(it->cpu, it->memory);

              ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 2.1.3 [" << it->name << "]");
              remaining_nodes.erase(it->name);
              ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 2.1.4 [" << it->name << "]");
              found_node_resources.push_back(node_it);
              ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 2.1.5 [" << it->name << "]");
            }
            break;
          case tug_resource_monitor::NodeInfo::ERROR_PID_NOT_FOUND: // nothing to do as remaining nodes will have this node in its set
            ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 2.1.6");
            break;
        }
      }
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3");
      if(received_first_msg_)
      {
        ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.1");
        for (std::set<std::string>::iterator it = remaining_nodes.begin(); it != remaining_nodes.end(); ++it)
          reportError(*it, "not_running_" + *it, "The node with the name '" + *it + "' is not running");
        ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.2");
        for (std::vector<std::map<std::string, NodeResource>::iterator>::iterator it = found_node_resources.begin();
             it != found_node_resources.end(); ++it)
        {
          ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.3");
          std::string name = (*it)->first;
          ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.4 " << name);
          std::vector<std::string> states = (*it)->second.estimateStates();
          ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.5 " << " with number of states: " << states.size());
          if(states.empty())
          {
            reportError(name, "no_state_" + name, "For the node with the name '" + name + "' no state could be estimated");
          }
          else
          {
            reportStates(name, states);
            ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.5");
          }
        }
      }
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 4");
      received_first_msg_ = true;
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 5");
    }

}


PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ResourcesPlugin, tug_observers_cpp::ObserverPluginBase)