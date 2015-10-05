//
// Created by clemens on 22.09.15.
//

#include <tug_timeout_observer/TimeoutSubs.h>
#include <tug_yaml/ProcessYaml.h>

TimeoutSubs::TimeoutSubs(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base) : plugin_base_(plugin_base)
{
  topic_ = ProcessYaml::getValue<std::string>("name", params);
  sub_ = plugin_base_->subscribe(topic_, 10, &TimeoutSubs::cb, this);

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
    std::vector<std::string> callerid_list = ProcessYaml::getValue<std::vector<std::string> >("callerid", callerids_params[i]);
    if(callerid_list.empty())
    {
      default_config_ = boost::make_shared<XmlRpc::XmlRpcValue>(callerids_params[i]);
    }
    else
    {
      for (size_t j = 0; j < callerid_list.size(); ++j)
      {
        std::string caller_id = callerid_list[j];
        std::map<std::string, boost::shared_ptr<TimeoutBase> >::iterator it = callerids_config_.find(caller_id);
        if(it == callerids_config_.end())
        {
          boost::shared_ptr<TimeoutBase> base = boost::make_shared<TimeoutBase>(topic_, callerids_params[i], plugin_base_);
          callerids_config_.insert(std::make_pair(caller_id, base));
        }
      }
    }
  }
}

void TimeoutSubs::cb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
{
  boost::shared_ptr<ros::M_string> connection_header = msg_event.getConnectionHeaderPtr();
  if(connection_header)
  {
    boost::mutex::scoped_lock the_lock(bases_lock_);
    ros::M_string::const_iterator it = connection_header->find("callerid");
    if(it == connection_header->end())
      ROS_ERROR("connection header has no caller id");
    else
    {
      std::string caller_id = it->second;
      boost::shared_ptr<TimeoutBase> base;
      std::map<std::string, boost::shared_ptr<TimeoutBase> >::iterator it = callerids_config_.find(caller_id);
      if(it == callerids_config_.end())
      {
        // caller id is not bound to a base we can only use the default config if it exists
        if(default_config_)
        {
          base = boost::make_shared<TimeoutBase>(topic_, *default_config_, plugin_base_);
          callerids_config_.insert(std::make_pair(caller_id, base));
        }
        else
        {
          ROS_WARN("unknown caller id and no default merge method");
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
