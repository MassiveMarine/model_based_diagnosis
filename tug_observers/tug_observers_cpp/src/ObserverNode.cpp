//
// Created by clemens on 03.08.15.
//

#include <tug_observers_cpp/ObserverNode.h>

namespace tug_observers_cpp
{
    ObserverNode::ObserverNode(ros::NodeHandle nh) : nh_(nh)
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

      for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
      {
        XmlRpc::XmlRpcValue & param = it->second;

        if (!param.hasMember("type"))
          throw std::runtime_error("/" + it->first + " has no 'type' parameter");

        ObserverPluginBasePtr new_plugin = plugin_manager_.loadPlugin(it->first, param["type"]);
        new_plugin->initialize(ros::NodeHandle(nh_, it->first));
      }
    }

    void ObserverNode::startPlugins()
    {
      Observers observers = plugin_manager_.getPluginList();
      for(Observers::iterator it = observers.begin(); it != observers.end(); ++it)
      {
        it->instance->startPlugin();
      }
    }
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "tug_robot_control_node");
    ros::NodeHandle nh;
    tug_observers_cpp::ObserverNode node(nh);
    node.initPlugins();
    node.startPlugins();
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