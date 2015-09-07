//
// Created by clemens on 04.08.15.
//

#include <tug_observer_plugins_cpp/NodeResource.h>
#include <tug_observer_plugins_cpp/ProcessYaml.h>
#include <ros/ros.h>
#include <boost/make_shared.hpp>

namespace tug_observer_plugins_cpp
{

    NodeResource::NodeResource(XmlRpc::XmlRpcValue &value)
    {
      ROS_DEBUG("[NodeResource::NodeResource] 1");
      if (!value.hasMember("cpu_filter"))
      {
        ROS_DEBUG("No cpu_filter for node given for resource plugin");
        throw std::runtime_error("No cpu_filter node given for resource plugin");
      }
      ROS_DEBUG("[NodeResource::NodeResource] 2");
      XmlRpc::XmlRpcValue cpu_filter_params = value["cpu_filter"];
      ROS_DEBUG("[NodeResource::NodeResource] 3");
      cpu_filter_ = boost::make_shared<Filter<double> >(cpu_filter_params);

      ROS_DEBUG("[NodeResource::NodeResource] 4");
      if (!value.hasMember("mem_filter"))
      {
        ROS_DEBUG("No mem_filter for node given for resource plugin");
        throw std::runtime_error("No mem_filter for node given for resource plugin");
      }
      ROS_DEBUG("[NodeResource::NodeResource] 5");
      XmlRpc::XmlRpcValue mem_filter_params = value["mem_filter"];
      ROS_DEBUG("[NodeResource::NodeResource] 6");
      memory_filter_ = boost::make_shared<Filter<unsigned long> >(mem_filter_params);

      ROS_DEBUG("[NodeResource::NodeResource] 7");
      if (!value.hasMember("states"))
      {
        ROS_DEBUG("No states for node given for resource plugin");
        throw std::runtime_error("No states for node given for resource plugin");
      }
      ROS_DEBUG("[NodeResource::NodeResource] 8");
      XmlRpc::XmlRpcValue params = value["states"];
      for (int i = 0; i < params.size(); ++i)
        states_.push_back(NodeResourceState(params[i]));
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
      FilteState<double> cpu = cpu_filter_->getFilteState();
      ROS_DEBUG("NodeResource::estimateStates 2");
      FilteState<unsigned long> memory = memory_filter_->getFilteState();
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
