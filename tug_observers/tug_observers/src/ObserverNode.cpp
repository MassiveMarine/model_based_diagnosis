/*
This file is part of the software provided by the tug ais group
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tug_observers/ObserverNode.h>
#include <string>
#include <tug_yaml/ProcessYaml.h>

namespace tug_observers
{
    ObserverNode::ObserverNode(ros::NodeHandle nh) : plugin_manager_("tug_observers",
                                                                     "tug_observers::ObserverPluginBase"), nh_(nh)
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
        XmlRpc::XmlRpcValue &param = params[i];

        if (!param.hasMember("type"))
          throw std::runtime_error("/" + static_cast<std::string>(params[i]) + " has no 'type' parameter");

        std::string type = static_cast<std::string>(param["type"]);
        ROS_DEBUG_STREAM("loading observer of type " << type);
        ObserverPluginBasePtr new_plugin = plugin_manager_.loadPlugin(type, type);

        if (param.hasMember("start_up_time"))
        {
          ROS_DEBUG_STREAM("has start up time will used it to wait");
          double start_up_time = ProcessYaml::getValue<double>("start_up_time", param);
          if (start_up_time <= 0.)
            throw std::runtime_error(
                    "start up time for /" + static_cast<std::string>(param) + " must be posive time in seconds");

          new_plugin->setStartUpTime(start_up_time);
        }

        ROS_DEBUG_STREAM("initialize observer of type " << type);
        new_plugin->initialize(params[i]);
        ROS_DEBUG_STREAM("done prepearing of type " << type);
      }
    }

    void ObserverNode::startPlugins()
    {
      ROS_DEBUG_STREAM("[ObserverNode::startPlugins] 1");
      Observers observers = plugin_manager_.getPluginList();
      ROS_DEBUG_STREAM("[ObserverNode::startPlugins] 2");
      for (Observers::iterator it = observers.begin(); it != observers.end(); ++it)
      {
        ROS_DEBUG_STREAM("[ObserverNode::startPlugins] 2.1");
        it->instance->startPlugin();
      }
      ROS_DEBUG_STREAM("[ObserverNode::startPlugins] 2.2");
    }
}  // namespace tug_observers

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "tug_observers_node");
    ros::NodeHandle nh("~");
    tug_observers::ObserverNode node(nh);
    ROS_INFO("init plugins");
    node.initPlugins();
    ROS_INFO("start plugins");
    node.startPlugins();
    ROS_INFO("plugins up and running");
    ros::spin();
  }
  catch (std::exception &ex)
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
  return 1;
}
