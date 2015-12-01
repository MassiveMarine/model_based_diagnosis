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

#include <tug_resource_observer/NodeResource.h>
#include <tug_yaml/ProcessYaml.h>
#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <tug_observers/Observation.h>
#include <vector>

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
      memory_filter_ = boost::make_shared<Filter<uint64_t > >(mem_filter_params);

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

    void NodeResource::update(double cpu, uint64_t  memory)
    {
      ROS_DEBUG_STREAM("NodeResource::update called with " << cpu << " and " << memory);
      cpu_filter_->update(cpu);
      memory_filter_->update(memory);
    }

    std::vector<Observation> NodeResource::estimateStates()
    {
      ROS_DEBUG("NodeResource::estimateStates 1");
      FilteState<double> cpu = cpu_filter_->getFilteState();
      ROS_DEBUG("NodeResource::estimateStates 2");
      FilteState<uint64_t > memory = memory_filter_->getFilteState();
      ROS_DEBUG("NodeResource::estimateStates 3");
      std::vector<Observation> result;
      for (std::vector<NodeResourceState>::iterator it = states_.begin(); it != states_.end(); ++it)
      {
        ROS_DEBUG("NodeResource::estimateStates 3.1");
        if (it->conformsState(cpu, memory))
          result.push_back(Observation(it->getName(), it->getNumber()));
        ROS_DEBUG("NodeResource::estimateStates 3.2");
      }
      ROS_DEBUG("NodeResource::estimateStates 4");
      return result;
    }
}  // namespace tug_observer_plugins_cpp
