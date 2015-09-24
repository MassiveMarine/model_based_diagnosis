//
// Created by clemens on 05.08.15.
//

#include <tug_observers/ObserverInfoSender.h>
#include <tug_observers_msgs/observer_info.h>

ObserverInfoSender::ObserverInfoSender() : wait_period_(1 * 1000 * 1000)
{
  ROS_DEBUG_STREAM("constructor of observer info sender called");
  ros::NodeHandle private_nh("~");
  double rate;
  private_nh.param<double>("info_rate", rate, 1.0);
  wait_period_ = boost::posix_time::microseconds(1./rate * 1000. * 1000.);

  info_pub_ = nh_.advertise<tug_observers_msgs::observer_info>("/observers/info", 1);
  background_thread_ = boost::thread(&ObserverInfoSender::run, this);
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

void ObserverInfoSender::run()
{
  boost::asio::io_service io_service;
  boost::asio::deadline_timer timer(io_service);

  while(ros::ok())
  {
    timer.expires_from_now(wait_period_);
    {
      boost::mutex::scoped_lock the_lock(observer_infos_mutex_);
      tug_observers_msgs::observer_info msg;
      for (std::map<ObserverInfo, std::pair<std::vector<std::string>, ros::Time> >::iterator it = current_observer_infos_.begin();
           it != current_observer_infos_.end(); ++it)
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

    timer.wait();
  }
}

void ObserverInfoSender::updateInfo(ObserverInfo info, std::vector<std::string> states, ros::Time time_of_occurence)
{
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);
  current_observer_infos_.erase(info);
  current_observer_infos_.insert(std::make_pair(info, std::make_pair(states, time_of_occurence)));
}

void ObserverInfoSender::removeInfo(std::string type, std::string resource)
{
  getInstance().removeInfo(ObserverInfo(type, resource));
}

void ObserverInfoSender::removeInfo(ObserverInfo info)
{
  boost::mutex::scoped_lock the_lock(observer_infos_mutex_);
  current_observer_infos_.erase(info);
}
