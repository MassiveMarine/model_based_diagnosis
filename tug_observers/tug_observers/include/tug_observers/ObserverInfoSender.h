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
#include <tug_timers/Timer.h>

struct ObserverInfo
{
  std::string type;
  std::string resource;

  ObserverInfo(const std::string &_type, const std::string &_resource) : type(_type), resource(_resource)
  { }

  bool operator<(const ObserverInfo& other) const
  {
    if(type < other.type)
      return true;

    if(resource < other.resource)
      return true;

    return false;
  }
};

class ObserverInfoSender
{
  std::map<ObserverInfo, std::pair<std::vector<std::string>, ros::Time> > current_observer_infos_;
  ros::NodeHandle nh_;
  ros::Publisher info_pub_;
  boost::mutex observer_infos_mutex_;
  boost::shared_ptr<Timer> timer_;

  ObserverInfoSender();

  void run();
  void updateInfo(ObserverInfo info, std::vector<std::string> states, ros::Time time_of_occurence);
  void removeInfo(ObserverInfo info);

  public:
    static ObserverInfoSender& getInstance();
    static void sendInfo(std::string type, std::string resource, std::vector<std::string> states, ros::Time time_of_occurence);
    static void removeInfo(std::string type, std::string resource);
};


#endif //TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H
