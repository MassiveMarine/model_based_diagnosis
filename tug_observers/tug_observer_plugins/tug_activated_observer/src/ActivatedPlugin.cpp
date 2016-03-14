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

#include <tug_activated_observer/ActivatedPlugin.h>
#include <ros/ros.h>
#include <tug_yaml/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>
#include <string>
#include <vector>
#include <set>

namespace tug_observer_plugins_cpp
{
ActivatedPlugin::ActivatedPlugin() : ObserverPluginBase("activated")
{ }

void ActivatedPlugin::initialize(XmlRpc::XmlRpcValue params)
{
  ROS_DEBUG("[ActivatedPlugin::initialize] 1");
  if (!params.hasMember("nodes"))
  {
    ROS_DEBUG("No nodes given for resource plugin");
    throw std::runtime_error("No nodes given for resource plugin");
  }
  ROS_DEBUG("[ActivatedPlugin::initialize] 2");
  XmlRpc::XmlRpcValue nodes = params["nodes"];
  for (int i = 0; i < nodes.size(); ++i)
  {
    ROS_DEBUG("[ActivatedPlugin::initialize] 2.1");
    XmlRpc::XmlRpcValue &param = nodes[i];
    ROS_DEBUG("[ActivatedPlugin::initialize] 2.2");
    std::string name = ProcessYaml::getValue<std::string>("name", param);
    nodes_of_interrest_.insert(name);
  }
  ROS_DEBUG("[ActivatedPlugin::initialize] 3");

  std::string resource_topic = ProcessYaml::getValue<std::string>("resource_topic", params);
  resource_sub_ = subscribe(resource_topic, 1, &ActivatedPlugin::nodeInfoCallback, this);
}

void ActivatedPlugin::nodeInfoCallback(const tug_resource_monitor::NodeInfoArray::ConstPtr &msg)
{
  if (!isStartedUp())
    return;

  ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 1");
  std::set<std::string> remaining_nodes = nodes_of_interrest_;
  ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 2");
  for (NodeInfoArray::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
  {
    ROS_DEBUG_STREAM("ActivatedPlugin::nodeInfoCallback 2.1 [" << it->name << "]");
    switch (it->error)
    {
      case tug_resource_monitor::NodeInfo::NO_ERROR:
        {
          if (remaining_nodes.find(it->name) != remaining_nodes.end())
          {
            std::vector<Observation> states;
            states.push_back(Observation("ok", tug_observers_msgs::observation::GENERAL_OK));
            reportStates(it->name, states, msg->header.stamp);

            ROS_DEBUG_STREAM("ActivatedPlugin::nodeInfoCallback 2.1.3 [" << it->name << "]");
            remaining_nodes.erase(it->name);
          }
        }
        break;
      case tug_resource_monitor::NodeInfo::ERROR_PID_NOT_FOUND:
        // nothing to do as remaining nodes will have this node in its set
        ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 2.1.6");
        break;
    }
  }
  ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 3");
  flush();
  ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 4");
  for (std::set<std::string>::iterator it = remaining_nodes.begin(); it != remaining_nodes.end(); ++it)
    reportError(*it, "not_running_" + *it, "The node with the name '" + *it + "' is not running",
                tug_observers_msgs::observation::NOT_AVAILABLE, msg->header.stamp);
}

}  // namespace tug_observer_plugins_cpp

PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ActivatedPlugin, tug_observers::ObserverPluginBase)
