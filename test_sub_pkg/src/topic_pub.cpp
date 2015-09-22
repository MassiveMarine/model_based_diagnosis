//
// Created by clemens on 22.09.15.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_pub");

  ros::NodeHandle nh;

  std::vector<ros::Publisher> publishers;

  for(size_t i = 0; i < 100; ++i)
  {
    std::stringstream topic_name;
    topic_name << "/topic_" << i;
    publishers.push_back(nh.advertise<std_msgs::String>(topic_name.str(), 1));
  }

  ros::Rate loop_rate(10);
  size_t count = 0;
  while(ros::ok())
  {
    std::stringstream msg_stream;
    msg_stream << "the msgs" << count;

    std_msgs::String msg;
    msg.data = msg_stream.str();

    for(size_t i = 0; i < 100; ++i)
      publishers[i].publish(msg);

    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
}