//
// Created by clemens on 03.08.15.
//

#include <tug_observers_cpp/ObserverNode.h>

namespace tug_observers_cpp
{
    ObserverNode::ObserverNode(ros::NodeHandle nh) : plugin_manager_("tug_observers_cpp", "tug_observers_cpp::ObserverPluginBase"), nh_(nh)
    { }

    void ObserverNode::initPlugins()
    {
      XmlRpc::XmlRpcValue params;
      nh_.getParam("setup", params);

      if (!params.valid())
      {
        ROS_ERROR("No Plugins given");
        throw std::runtime_error("No Plugins given");
      }

      for (int i = 0; i < params.size(); ++i)
      {
        XmlRpc::XmlRpcValue & param = params[i];

        if (!param.hasMember("type"))
          throw std::runtime_error("/" + static_cast<std::string>(params[i]) + " has no 'type' parameter");

        std::string type = static_cast<std::string>(param["type"]);
        ObserverPluginBasePtr new_plugin = plugin_manager_.loadPlugin(type, type);
        new_plugin->initialize(params[i]);
      }
    }

    void ObserverNode::startPlugins()
    {
      ROS_DEBUG("[ObserverNode::startPlugins] 1");
      Observers observers = plugin_manager_.getPluginList();
      ROS_DEBUG("[ObserverNode::startPlugins] 2");
      for(Observers::iterator it = observers.begin(); it != observers.end(); ++it)
      {
        ROS_DEBUG("[ObserverNode::startPlugins] 2.1");
        it->instance->startPlugin();
      }
      ROS_DEBUG("[ObserverNode::startPlugins] 2.2");
    }
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "tug_observers_node");
    ros::NodeHandle nh("~");
    tug_observers_cpp::ObserverNode node(nh);
    node.initPlugins();
    node.startPlugins();
    ros::spin();
  }
  catch (std::exception & ex)
  {
    std::cerr << "Uncaught exception in main: " << ex.what() << std::endl;
    return -1;
  }
  catch (...)
  {
    std::cerr << "Uncaught exception in main" << std::endl;
    throw;
    return -1;

  }
  return -1;
}
