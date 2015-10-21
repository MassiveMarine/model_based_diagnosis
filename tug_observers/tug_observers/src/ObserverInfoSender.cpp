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
  timeout_thread_ = boost::make_shared<Timeout>(boost::posix_time::seconds(1./rate), boost::bind(&ObserverInfoSender::executeFlush, this));

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

bool ObserverInfoSender::executeFlush()
{
  ROS_DEBUG_STREAM("execute flush called");
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);
  ROS_DEBUG_STREAM("check if there are observations to publish");
  if(current_obser_info_.observation_infos.empty())
    return true;
  ROS_DEBUG_STREAM("publish observer info");
  info_pub_.publish(current_obser_info_);
  current_obser_info_.observation_infos.clear();

  ROS_DEBUG_STREAM("return with true");
  return true;
}

void ObserverInfoSender::flushIntern()
{
  timeout_thread_->set();
  executeFlush();
}

void ObserverInfoSender::sendInfo(std::string resource, std::string type, std::vector<Observation> observations, ros::Time time_of_occurence)
{
  getInstance().sendInfoIntern(resource, type, observations, time_of_occurence);
}

void ObserverInfoSender::flush()
{
  getInstance().flushIntern();
}