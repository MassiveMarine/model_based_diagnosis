//
// Created by clemens on 05.08.15.
//

#include <tug_observers_cpp/ObserverInfoSender.h>
#include <tug_observers_msgs/observer_info.h>

ObserverInfoSender::ObserverInfoSender()
{
  ros::NodeHandle private_nh("~");
  double period;
  private_nh.param<double>("info_period", period, 1.0);

  info_pub_ = nh_.advertise<tug_observers_msgs::observer_info>("/observers/info", 1);
  timer_ = nh_.createTimer(ros::Duration(1.0), &ObserverInfoSender::timerCallback, this);
}

ObserverInfoSender& ObserverInfoSender::getInstance()
{
  static ObserverInfoSender instance;

  return instance;
}

void ObserverInfoSender::sendInfo(std::string type, std::string resource, std::vector<std::string> states, ros::Time time_of_occurence)
{
  getInstance().updateInfo(ObserverInfo(type, resource), states, time_of_occurence);
}

void ObserverInfoSender::timerCallback(const ros::TimerEvent &timer_evnt)
{
  tug_observers_msgs::observer_info msg;
  for(std::map<ObserverInfo, std::pair<std::vector<std::string>, ros::Time> >::iterator it = current_observer_infos_.begin(); it != current_observer_infos_.end(); ++it)
  {
    tug_observers_msgs::resource_info info;
    info.resource = it->first.resource;
    info.type = it->first.type;
    info.header.stamp = it->second.second;
    info.states = it->second.first;
    msg.resource_infos.push_back(info);
  }

  info_pub_.publish(msg);
}

void ObserverInfoSender::updateInfo(ObserverInfo info, std::vector<std::string> states, ros::Time time_of_occurence)
{
  current_observer_infos_.erase(info);
  current_observer_infos_.insert(std::make_pair(info, std::make_pair(states, time_of_occurence)));
}

void ObserverInfoSender::removeInfo(std::string type, std::string resource)
{
  getInstance().removeInfo(ObserverInfo(type, resource));
}

void ObserverInfoSender::removeInfo(ObserverInfo info)
{
  current_observer_infos_.erase(info);
}
