//
// Created by clemens on 24.09.15.
//

#include <tug_velocity_observer/VelocityChecker.h>

VelocityChecker::VelocityChecker()
{ }

void VelocityChecker::init(XmlRpc::XmlRpcValue params, tug_observers::ObserverPluginBase* plugin_base)
{
  ROS_ERROR("VelocityChecker::init 1");
  if (!params.hasMember("correlations"))
  {
    ROS_ERROR("No correlations for velocity plugin defined");
    throw std::runtime_error("No correlations for velocity plugin defined");
  }
  ROS_ERROR("VelocityChecker::init 2");
  XmlRpc::XmlRpcValue correlations_params = params["correlations"];
  for (int i = 0; i < correlations_params.size(); ++i)
    observers_.push_back(boost::make_shared<VelocityObserver>(correlations_params[i], plugin_base));
  ROS_ERROR("VelocityChecker::init 3");
}

std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > VelocityChecker::velocityObservations()
{
  std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > result;
  for(std::vector<boost::shared_ptr<VelocityObserver> >::iterator it = observers_.begin(); it != observers_.end(); ++it)
  {
    std::pair<bool, std::vector<Observation> > states = (*it)->estimateStates();
    if (states.first)
    {
      ROS_DEBUG_STREAM("got estimated states: " << states.second.size());
      result.push_back(
              boost::make_tuple<std::string, std::vector<Observation>, ros::Time>((*it)->getName(), states.second,
                                                                                  (*it)->getCurrentFilterTime()));
    }
    else
      ROS_DEBUG("got no estimated states");
  }

  return result;
}
