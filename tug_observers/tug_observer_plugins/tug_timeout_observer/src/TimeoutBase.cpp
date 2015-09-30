//
// Created by clemens on 22.09.15.
//

#include <tug_timeout_observer/TimeoutBase.h>
#include <tug_observers_msgs/observation.h>

TimeoutBase::TimeoutBase(std::string topic, XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base) : plugin_base_(plugin_base)
{
  std::stringstream name_stream;
  name_stream << topic << " " << params["callerid"];
  name_ = name_stream.str();

  if (!params.hasMember("timeout"))
  {
    ROS_ERROR("No timeout for timeout plugin defined");
    throw std::runtime_error("No timeout for timeout plugin defined");
  }
  timeout_ = ProcessYaml::getValue<double>("timeout", params);

  if (!params.hasMember("max_timeouts_in_a_row"))
  {
    ROS_ERROR("No maximum number of timeouts for timeout plugin defined");
    throw std::runtime_error("No maximum number of timeouts for timeout plugin defined");
  }
  max_timeouts_in_a_row_ = ProcessYaml::getValue<int>("max_timeouts_in_a_row", params);
  remaining_timeouts_ = max_timeouts_in_a_row_;

  timeout_thread_ = boost::make_shared<Timeout>(boost::posix_time::seconds(timeout_), boost::bind(&TimeoutBase::timeout_callback, this));
}

void TimeoutBase::update()
{
  timeout_thread_->set();
  remaining_timeouts_ = max_timeouts_in_a_row_;
}

bool TimeoutBase::timeout_callback()
{
  if (remaining_timeouts_ <= 0)
    return false;

  remaining_timeouts_ -= 1;

  plugin_base_->reportError(name_, "no_state_" + name_, "For the node with the name '" + name_ + "' no state could be estimated", tug_observers_msgs::observation::NO_AVAILABLE, ros::Time::now());

  return true;
}