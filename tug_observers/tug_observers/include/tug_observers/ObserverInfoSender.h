//
// Created by clemens on 05.08.15.
//

#ifndef TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H
#define TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <tug_time/Timer.h>
#include <tug_observers/Observation.h>
#include <tug_observers_msgs/observer_info.h>

class ObserverInfoSender
{
  ros::NodeHandle nh_;
  ros::Publisher info_pub_;
  boost::mutex observer_infos_mutex_;
  tug_observers_msgs::observer_info current_obser_info_;

  ObserverInfoSender();
  void sendInfoIntern(std::string resource, std::string type, std::vector<Observation> observations, ros::Time time_of_occurence);
  void flushIntern();

  public:
    static ObserverInfoSender& getInstance();
    static void sendInfo(std::string resource, std::string type, std::vector<Observation> observations, ros::Time time_of_occurence);
    static void flush();
};


#endif //TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H
