//
// Created by clemens on 05.08.15.
//

#ifndef TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H
#define TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

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
  std::map<ObserverInfo, std::vector<std::string> > current_observer_infos_;
  ros::NodeHandle nh_;
  ros::Publisher info_pub_;
  ros::Timer timer_;

  ObserverInfoSender();

  void timerCallback(const ros::TimerEvent& timer_evnt);
  void updateInfo(ObserverInfo info, std::vector<std::string> states);
  void removeInfo(ObserverInfo info);

  public:
    static ObserverInfoSender& getInstance();
    static void sendInfo(std::string type, std::string resource, std::vector<std::string> states);
    static void removeInfo(std::string type, std::string resource);
};


#endif //TUG_OBSERVERS_CPP_OBSERVERINFOSENDER_H
