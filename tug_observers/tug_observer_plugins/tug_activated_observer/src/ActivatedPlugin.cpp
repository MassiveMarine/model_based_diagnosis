//
// Created by clemens on 04.08.15.
//

#include <tug_activated_observer/ActivatedPlugin.h>
#include <ros/ros.h>
#include <tug_yaml/ProcessYaml.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observers_msgs/observation.h>

namespace tug_observer_plugins_cpp
{
    ActivatedPlugin::ActivatedPlugin() : ObserverPluginBase("activated")
    {

    }

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
              if(remaining_nodes.find(it->name) != remaining_nodes.end())
              {
                std::vector<Observation> states;
                states.push_back(Observation("ok", tug_observers_msgs::observation::GENERAL_OK));
                reportStates(it->name, states, msg->header.stamp);

                ROS_DEBUG_STREAM("ActivatedPlugin::nodeInfoCallback 2.1.3 [" << it->name << "]");
                remaining_nodes.erase(it->name);
              }
            }
            break;
          case tug_resource_monitor::NodeInfo::ERROR_PID_NOT_FOUND: // nothing to do as remaining nodes will have this node in its set
            ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 2.1.6");
            break;
        }
      }
      ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 3");
      flush();
      ROS_DEBUG("ActivatedPlugin::nodeInfoCallback 4");
      for (std::set<std::string>::iterator it = remaining_nodes.begin(); it != remaining_nodes.end(); ++it)
        reportError(*it, "not_running_" + *it, "The node with the name '" + *it + "' is not running", tug_observers_msgs::observation::NOT_AVAILABLE, msg->header.stamp);

    }

}


PLUGINLIB_EXPORT_CLASS(tug_observer_plugins_cpp::ActivatedPlugin, tug_observers::ObserverPluginBase)
