//
// Created by clemens on 22.09.15.
//

#include <tug_hz_observer/HzSubs.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>

HzSubs::HzSubs(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base)
{
  topic_ = ProcessYaml::getValue<std::string>("name", params);
  sub_ = plugin_base->subscribe(topic_, 1, &HzSubs::cb, this);

  if (!params.hasMember("callerids"))
  {
    ROS_ERROR("No callerids for hz plugin defined");
    throw std::runtime_error("No callerids for hz plugin defined");
  }
  XmlRpc::XmlRpcValue callerids_params = params["callerids"];
  for (int i = 0; i < callerids_params.size(); ++i)
  {
    boost::shared_ptr<HzMergedBases> tmp_merged_base = boost::make_shared<HzMergedBases>(topic_, callerids_params[i], plugin_base);
    merged_bases_.insert(tmp_merged_base);

    if (!callerids_params[i].hasMember("callerid"))
    {
      ROS_ERROR("No callerid for hz plugin defined");
      throw std::runtime_error("No callerid for hz plugin defined");
    }
    std::vector<std::string> callerid_list = ProcessYaml::getValue<std::vector<std::string> >("callerid", callerids_params[i]);
    if(callerid_list.empty())
    {
      default_config_ = boost::make_shared<XmlRpc::XmlRpcValue>(callerids_params[i]);
      default_merged_bases_ = tmp_merged_base;
    }
    else
    {
      for (size_t j = 0; j < callerid_list.size(); ++j)
      {
        std::string caller_id = callerid_list[j];
        std::map<std::string, boost::shared_ptr<HzBase> >::iterator it = callerids_config_.find(caller_id);
        if(it == callerids_config_.end())
        {
          boost::shared_ptr<HzBase> base = boost::make_shared<HzBase>(callerids_params[i]);
          callerids_config_.insert(std::make_pair(caller_id, base));
          tmp_merged_base->addBase(base);
        }
      }
    }
  }
}

void HzSubs::cb(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
{
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();
  if(connection_header)
  {
    boost::mutex::scoped_lock the_lock(bases_lock_);
    ros::M_string::const_iterator it = connection_header->find("callerid");
    if(it == connection_header->end())
      ROS_ERROR("connection header has no caller id");
    else
    {
      std::string caller_id = it->second;
      boost::shared_ptr<HzBase> base;
      std::map<std::string, boost::shared_ptr<HzBase> >::iterator it = callerids_config_.find(caller_id);
      if(it == callerids_config_.end())
      {
        // caller id is not bound to a base we can only use the default config if it exists
        if(default_config_ && default_merged_bases_)
        {
          base = boost::make_shared<HzBase>(*default_config_);
          callerids_config_.insert(std::make_pair(caller_id, base));
          default_merged_bases_->addBase(base);
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

void HzSubs::sendResourceInfo()
{
  for(std::set<boost::shared_ptr<HzMergedBases> >::iterator it = merged_bases_.begin(); it != merged_bases_.end(); ++it)
    (*it)->sendResourceInfo();
}