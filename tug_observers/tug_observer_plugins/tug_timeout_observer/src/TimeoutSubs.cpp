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

#include <tug_timeout_observer/TimeoutSubs.h>
#include <tug_yaml/ProcessYaml.h>
#include <map>
#include <string>
#include <vector>

TimeoutSubs::TimeoutSubs(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base) :
        plugin_base_(plugin_base)
{
  ROS_DEBUG_STREAM("TimeoutSubs::TimeoutSubs called");
  topic_ = ProcessYaml::getValue<std::string>("name", params);

  if (!params.hasMember("callerids"))
  {
    ROS_ERROR("No callerids for timeout plugin defined");
    throw std::runtime_error("No callerids for timeout plugin defined");
  }
  XmlRpc::XmlRpcValue callerids_params = params["callerids"];
  for (int i = 0; i < callerids_params.size(); ++i)
  {
    if (!callerids_params[i].hasMember("callerid"))
    {
      ROS_ERROR("No callerid for timeout plugin defined");
      throw std::runtime_error("No callerid for timeout plugin defined");
    }

    std::vector<std::string> callerid_list = ProcessYaml::getValue<std::vector<std::string> >("callerid",
                                                                                              callerids_params[i]);
    if (callerid_list.empty())
    {
      default_base_ = boost::make_shared<TimeoutBase>(topic_, callerids_params[i], plugin_base_);
    }
    else
    {
      for (size_t j = 0; j < callerid_list.size(); ++j)
      {
        std::string caller_id = callerid_list[j];
        std::map<std::string, boost::shared_ptr<TimeoutBase> >::iterator it = callerids_config_.find(caller_id);
        if (it == callerids_config_.end())
        {
          boost::shared_ptr<TimeoutBase> base = boost::make_shared<TimeoutBase>(topic_, callerids_params[i],
                                                                                plugin_base_);
          callerids_config_.insert(std::make_pair(caller_id, base));
        }
      }
    }
  }

  sub_ = plugin_base_->subscribe(topic_, 10, &TimeoutSubs::cb, this);
}

void TimeoutSubs::cb(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event)
{
  boost::shared_ptr<ros::M_string> connection_header = msg_event.getConnectionHeaderPtr();
  if (connection_header)
  {
    boost::mutex::scoped_lock the_lock(bases_lock_);
    ros::M_string::const_iterator it = connection_header->find("callerid");
    if (it == connection_header->end())
      ROS_ERROR("connection header has no caller id");
    else
    {
      std::string caller_id = it->second;
      boost::shared_ptr<TimeoutBase> base;
      std::map<std::string, boost::shared_ptr<TimeoutBase> >::iterator it = callerids_config_.find(caller_id);
      if (it == callerids_config_.end())
      {
        // caller id is not bound to a base we can only use the default config if it exists
        if (default_base_)
        {
          base = default_base_;
        }
        else
        {
          ROS_WARN("unknown caller id and no default base");
          return;
        }
      }
      else
        base = it->second;

      base->update();
    }
  }
  else
    ROS_ERROR("got message without header");
}
