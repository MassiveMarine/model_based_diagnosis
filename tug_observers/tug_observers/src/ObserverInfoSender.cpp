//
// Created by clemens on 05.08.15.
//

#include <tug_observers/ObserverInfoSender.h>

ObserverInfoSender::ObserverInfoSender()
{
  ROS_DEBUG_STREAM("constructor of observer info sender called");
  ros::NodeHandle private_nh("~");
  double rate;
  private_nh.param<double>("info_rate", rate, 1.0);

  info_pub_ = nh_.advertise<tug_observers_msgs::observer_info>("/observers/info", 1);
}

ObserverInfoSender& ObserverInfoSender::getInstance()
{
  static ObserverInfoSender instance;

  return instance;
}

void ObserverInfoSender::sendInfoIntern(std::string resource, std::string type, std::vector<Observation> observations, ros::Time time_of_occurence)
{
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);

  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = time_of_occurence;
  observation_info.resource = resource;
  observation_info.type = type;

  for(size_t i = 0; i < observations.size(); ++i)
    observation_info.observation.push_back(observations[i].toMsg());

  current_obser_info_.observation_infos.push_back(observation_info);
}

void ObserverInfoSender::flushIntern()
{
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);
  info_pub_.publish(current_obser_info_);
  current_obser_info_.observation_infos.clear();
}

void ObserverInfoSender::sendInfo(std::string resource, std::string type, std::vector<Observation> observations, ros::Time time_of_occurence)
{
  getInstance().sendInfo(resource, type, observations, time_of_occurence);
}

void ObserverInfoSender::flush()
{
  getInstance().flush();
}