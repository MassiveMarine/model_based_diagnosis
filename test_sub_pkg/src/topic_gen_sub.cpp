//
// Created by clemens on 22.09.15.
//

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <vector>
#include <boost/function.hpp>

void theCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
{
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();
  if(connection_header)
  {
    std::stringstream connection_header_stream;
    for(ros::M_string::const_iterator it = connection_header->begin(); it != connection_header->end(); ++it)
    {
      connection_header_stream  << "[" << it->first << "]='" << it->second << "'";
    }
    //ROS_ERROR_STREAM("got connection header with info: " << connection_header_stream.str());
  }
  else
    ROS_ERROR("got message without header");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_gen_sub");

  ros::NodeHandle nh;

  std::vector<ros::Subscriber> subsriber;

  for(size_t i = 0; i < 100; ++i)
  {
    std::stringstream topic_name;
    topic_name << "/topic_" << i;
    subsriber.push_back(nh.subscribe(topic_name.str(), 1, theCallback));
  }

  ros::spin();
}