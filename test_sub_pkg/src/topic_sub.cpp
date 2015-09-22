//
// Created by clemens on 22.09.15.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <boost/function.hpp>

void theCallback(size_t index, const std_msgs::String::ConstPtr& msg)
{
  //ROS_ERROR_STREAM("got msg at " << index << " with info " << msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_sub");

  ros::NodeHandle nh;

  std::vector<ros::Subscriber> subsriber;

  for(size_t i = 0; i < 100; ++i)
  {
    std::stringstream topic_name;
    topic_name << "/topic_" << i;
    subsriber.push_back(nh.subscribe(topic_name.str(), 1, boost::function<void(const std_msgs::String::ConstPtr&)>(boost::bind(theCallback, i, _1))));
  }

  ros::spin();
}