/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tug_resource_observer/ResourcesPlugin.h>
#include <ros/ros.h>
#include <tug_yaml/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>
#include <map>
#include <string>
#include <vector>
#include <set>

namespace tug_observer_plugins_cpp
{
    ResourcesPlugin::ResourcesPlugin() : ObserverPluginBase("resources"), received_first_msg_(false)
    { }

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

      std::string resource_topic = ProcessYaml::getValue<std::string>("resource_topic", params);
      resource_sub_ = subscribe(resource_topic, 1, &ResourcesPlugin::nodeInfoCallback, this);
    }

    void ResourcesPlugin::nodeInfoCallback(const tug_resource_monitor::NodeInfoArray::ConstPtr &msg)
    {
      if (!isStartedUp())
        return;

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
          case tug_resource_monitor::NodeInfo::ERROR_PID_NOT_FOUND:
            // nothing to do as remaining nodes will have this node in its set
            ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 2.1.6");
            break;
        }
      }
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3");
      if (received_first_msg_)
      {
        ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.1");
        for (std::set<std::string>::iterator it = remaining_nodes.begin(); it != remaining_nodes.end(); ++it)
          reportError(*it, "not_running_" + *it, "The node with the name '" + *it + "' is not running",
                      tug_observers_msgs::observation::NOT_AVAILABLE, msg->header.stamp);
        ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.2");
        for (std::vector<std::map<std::string, NodeResource>::iterator>::iterator it = found_node_resources.begin();
             it != found_node_resources.end(); ++it)
        {
          ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.3");
          std::string name = (*it)->first;
          ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.4 " << name);
          std::vector<Observation> states = (*it)->second.estimateStates();
          ROS_DEBUG_STREAM("ResourcesPlugin::nodeInfoCallback 3.5 " << " with number of states: " << states.size());
          if (states.empty())
          {
            reportError(name, "no_state_" + name, "For the node with the name '" + name
                                                  + "' no state could be estimated",
                        tug_observers_msgs::observation::NO_STATE_FITS, msg->header.stamp);
          }
          else
          {
            reportStates(name, states, msg->header.stamp);
            ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 3.5");
          }
        }

        flush();
      }
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 4");
      received_first_msg_ = true;
      ROS_DEBUG("ResourcesPlugin::nodeInfoCallback 5");
    }

}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ResourcesPlugin, tug_observers::ObserverPluginBase)
