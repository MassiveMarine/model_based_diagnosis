//
// Created by clemens on 17.11.15.
//

#include <gtest/gtest.h>
#include <tug_testing/TestHelper.h>
#include <tug_observers/ObserverInfoSender.h>
#include <tug_observers_msgs/observer_info.h>

typedef TestHelper<tug_observers_msgs::observer_info> TestHelperObserverInfo;

void tmpCB(const typename tug_observers_msgs::observer_info::ConstPtr&)
{ }

TEST(ObserverInfoSender, check_publisher)
{
  ObserverInfoSender::getInstance();
  ros::NodeHandle nh;
  ros::Subscriber the_sub = nh.subscribe("/observers/info", 1.1, &tmpCB);
  sleep(1);
  EXPECT_GT(the_sub.getNumPublishers(), 0);
}

void test1HelperFunction()
{ }

TEST_F(TestHelperObserverInfo, send_observer_info_test1)
{
  init("/observers/info");
  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(&test1HelperFunction, 1);
  EXPECT_FALSE(msg_pair.second);
}

void sendInfoTestHelperFunction(tug_observers_msgs::observer_info msg_to_send)
{
  ROS_DEBUG("sendInfoTestHelperFunction called");
  for(size_t i = 0; i < msg_to_send.observation_infos.size(); ++i)
  {
    tug_observers_msgs::observation_info observation_info_to_send = msg_to_send.observation_infos[i];

    std::string resource = observation_info_to_send.resource;
    std::string type = observation_info_to_send.type;
    ros::Time time_of_occurence = observation_info_to_send.header.stamp;
    std::vector<Observation> observations;
    for(size_t j = 0; j < observation_info_to_send.observation.size(); ++j)
    {
      tug_observers_msgs::observation observation_to_send = observation_info_to_send.observation[j];

      Observation the_observation(observation_to_send.observation_msg, observation_to_send.verbose_observation_msg,
                                  observation_to_send.observation);

      observations.push_back(the_observation);
    }

    ROS_DEBUG("call ObserverInfoSender::sendInfo");
    ObserverInfoSender::sendInfo(resource, type, observations, time_of_occurence);
  }
}

void sendAndFlushInfoTestHelperFunction(tug_observers_msgs::observer_info msg_to_send)
{
  ROS_DEBUG("sendAndFlushInfoTestHelperFunction called");
  sendInfoTestHelperFunction(msg_to_send);
  ROS_DEBUG("call ObserverInfoSender::flush");
  ObserverInfoSender::flush();
}

bool compareMsgs(tug_observers_msgs::observer_info msg_send, tug_observers_msgs::observer_info msg_received)
{
  if(msg_send.observation_infos.size() != msg_received.observation_infos.size())
    return false;
  for(size_t i = 0; i < msg_send.observation_infos.size(); ++i)
  {
    tug_observers_msgs::observation_info observation_info_send = msg_send.observation_infos[i];
    tug_observers_msgs::observation_info observation_info_received = msg_received.observation_infos[i];

    if(observation_info_send.header.stamp != observation_info_received.header.stamp)
      return false;

    if(observation_info_send.type != observation_info_received.type)
      return false;

    if(observation_info_send.resource != observation_info_received.resource)
      return false;

    if(observation_info_send.observation.size() != observation_info_received.observation.size())
      return false;

    for(size_t j = 0; j < observation_info_send.observation.size(); ++j)
    {
      tug_observers_msgs::observation observation_send = observation_info_send.observation[j];
      tug_observers_msgs::observation observation_received = observation_info_received.observation[j];

      if(observation_send.observation_msg != observation_received.observation_msg)
        return false;

      if(observation_send.verbose_observation_msg != observation_received.verbose_observation_msg)
        return false;

      if(observation_send.observation != observation_received.observation)
        return false;
    }
  }

  return true;
}

TEST_F(TestHelperObserverInfo, send_observer_info_test2)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_FALSE(msg_pair.second);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test3)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test4)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test5)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test6)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test7)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test8)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test9)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test10)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test11)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test12)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test13)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test14)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test15)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test16)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test17)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test18)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test19)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test20)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test21)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test22)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test23)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test24)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test25)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test26)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test27)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test28)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test29)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test30)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test31)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test32)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test33)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test34)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test35)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test36)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test37)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test38)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test39)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test40)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test41)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test42)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test43)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test44)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test45)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test46)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test47)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test48)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test49)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test50)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test51)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test52)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test53)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test54)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test55)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test56)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test57)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test58)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test59)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation1;
  observation1.observation = 1;
  observation1.observation_msg = "b";
  observation1.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation1);
  tug_observers_msgs::observation observation2;
  observation2.observation = 2;
  observation2.observation_msg = "r";
  observation2.verbose_observation_msg = "q";
  observation_info.observation.push_back(observation2);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 0.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test60)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test61)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test62)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test63)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test64)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test65)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test66)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test67)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test68)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test69)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test70)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test71)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test72)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test73)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test74)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test75)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test76)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test77)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test78)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test79)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test80)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test81)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test82)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test83)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test84)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test85)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test86)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test87)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test88)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test89)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test90)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test91)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test92)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test93)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test94)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test95)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test96)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test97)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test98)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test99)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test100)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test101)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test102)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test103)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test104)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test105)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test106)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test107)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test108)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test109)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test110)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test111)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test112)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test113)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test114)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test115)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = 1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test116)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation1;
  observation1.observation = 1;
  observation1.observation_msg = "b";
  observation1.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation1);
  tug_observers_msgs::observation observation2;
  observation2.observation = 2;
  observation2.observation_msg = "r";
  observation2.verbose_observation_msg = "q";
  observation_info.observation.push_back(observation2);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test117)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test118)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test119)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test120)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test121)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test122)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test123)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test124)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test125)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test126)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test127)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test128)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test129)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test130)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test131)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test132)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test133)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test134)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test135)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test136)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test137)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test138)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test139)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test140)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation;
  observation.observation = -1;
  observation.observation_msg = "b";
  observation.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test141)
{
  init("/observers/info");
  tug_observers_msgs::observer_info msg_to_send;
  tug_observers_msgs::observation_info observation_info;
  tug_observers_msgs::observation observation1;
  observation1.observation = -1;
  observation1.observation_msg = "b";
  observation1.verbose_observation_msg = "c";
  observation_info.observation.push_back(observation1);
  tug_observers_msgs::observation observation2;
  observation2.observation = 2;
  observation2.observation_msg = "r";
  observation2.verbose_observation_msg = "q";
  observation_info.observation.push_back(observation2);
  observation_info.header.stamp = ros::Time::now();
  observation_info.resource = "a";
  observation_info.type = "a";
  msg_to_send.observation_infos.push_back(observation_info);

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_TRUE(msg_pair.second);
  EXPECT_PRED2(compareMsgs, msg_to_send, msg_pair.first);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_observer_info");
  return RUN_ALL_TESTS();
}
