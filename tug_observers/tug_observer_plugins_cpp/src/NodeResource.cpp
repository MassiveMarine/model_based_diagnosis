//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/NodeResource.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <ros/ros.h>
#include <tug_observer_plugins_cpp/filter/value_filter/ValueFilterFactory.h>

namespace tug_observer_plugins_cpp
{

    NodeResource::NodeResource(XmlRpc::XmlRpcValue &value)
    {
      ROS_DEBUG("[NodeResource::NodeResource] 1");
      if (!value.hasMember("states"))
      {
        ROS_DEBUG("No states for node given for resource plugin");
        throw std::runtime_error("No states for node given for resource plugin");
      }
      ROS_DEBUG("[NodeResource::NodeResource] 2");
      XmlRpc::XmlRpcValue params = value["states"];
      for (int i = 0; i < params.size(); ++i)
        states_.push_back(NodeResourceState(params[i]));
      ROS_DEBUG("[NodeResource::NodeResource] 3");
      if (!value.hasMember("cpu_filter"))
      {
        ROS_DEBUG("No cpu_filter for node given for resource plugin");
        throw std::runtime_error("No cpu_filter node given for resource plugin");
      }
      ROS_DEBUG("[NodeResource::NodeResource] 4");
      XmlRpc::XmlRpcValue cpu_filter_params = value["cpu_filter"];
      ROS_DEBUG("[NodeResource::NodeResource] 5");
      cpu_filter_ = ValueFilterFactory<double>::createFilter(ProcessYaml::getValue<std::string>("type", cpu_filter_params), cpu_filter_params);
      ROS_DEBUG("[NodeResource::NodeResource] 6");
      if (!value.hasMember("mem_filter"))
      {
        ROS_DEBUG("No mem_filter for node given for resource plugin");
        throw std::runtime_error("No mem_filter for node given for resource plugin");
      }
      ROS_DEBUG("[NodeResource::NodeResource] 7");
      XmlRpc::XmlRpcValue mem_filter_params = value["mem_filter"];
      ROS_DEBUG("[NodeResource::NodeResource] 8");
      memory_filter_ = ValueFilterFactory<unsigned long>::createFilter(ProcessYaml::getValue<std::string>("type", mem_filter_params), mem_filter_params);
      ROS_DEBUG("[NodeResource::NodeResource] 9");
    }

    void NodeResource::update(double cpu, unsigned long memory)
    {
      ROS_DEBUG_STREAM("NodeResource::update called with " << cpu << " and " << memory );
      cpu_filter_->update(cpu);
      memory_filter_->update(memory);
    }

    std::vector<std::string> NodeResource::estimateStates()
    {
      ROS_DEBUG("NodeResource::estimateStates 1");
      double cpu = cpu_filter_->getValue();
      ROS_DEBUG("NodeResource::estimateStates 2");
      unsigned long memory = memory_filter_->getValue();
      ROS_DEBUG("NodeResource::estimateStates 3");
      std::vector<std::string> result;
      for(std::vector<NodeResourceState>::iterator it = states_.begin(); it != states_.end(); ++it)
      {
        ROS_DEBUG("NodeResource::estimateStates 3.1");
        if(it->conformsState(cpu, memory))
          result.push_back(it->getName());
        ROS_DEBUG("NodeResource::estimateStates 3.2");
      }
      ROS_DEBUG("NodeResource::estimateStates 4");
      return result;
    }
}
