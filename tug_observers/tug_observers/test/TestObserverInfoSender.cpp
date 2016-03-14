/*
This file is part of the software provided by the tug ais group
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <tug_testing/PublisherTester.h>
#include <tug_observers/ObserverInfoSender.h>
#include <tug_observers_msgs/observer_info.h>
#include <utility>
#include <string>
#include <vector>

class TestHelperObserverInfo : public ::testing::Test
{
    PublisherTester<tug_observers_msgs::observer_info> pub_tester_;
protected:
    TestHelperObserverInfo() : pub_tester_("/observers/info")
    { }

    virtual ~TestHelperObserverInfo()
    { }

    std::pair<tug_observers_msgs::observer_info, bool> getMessage(boost::function<void()> function_to_call,
                                                                  double time_to_wait)
    {
      return pub_tester_.getMessage(function_to_call, time_to_wait);
    };
};


void tmpCB(const typename tug_observers_msgs::observer_info::ConstPtr &)
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
  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(&test1HelperFunction, 1);
  EXPECT_FALSE(msg_pair.second);
}

void sendInfoTestHelperFunction(tug_observers_msgs::observer_info msg_to_send)
{
  ROS_DEBUG("sendInfoTestHelperFunction called");
  for (size_t i = 0; i < msg_to_send.observation_infos.size(); ++i)
  {
    tug_observers_msgs::observation_info observation_info_to_send = msg_to_send.observation_infos[i];

    std::string resource = observation_info_to_send.resource;
    std::string type = observation_info_to_send.type;
    ros::Time time_of_occurence = observation_info_to_send.header.stamp;
    std::vector<Observation> observations;
    for (size_t j = 0; j < observation_info_to_send.observation.size(); ++j)
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
  if (msg_send.observation_infos.size() != msg_received.observation_infos.size())
    return false;
  for (size_t i = 0; i < msg_send.observation_infos.size(); ++i)
  {
    tug_observers_msgs::observation_info observation_info_send = msg_send.observation_infos[i];
    tug_observers_msgs::observation_info observation_info_received = msg_received.observation_infos[i];

    if (observation_info_send.header.stamp != observation_info_received.header.stamp)
      return false;

    if (observation_info_send.type != observation_info_received.type)
      return false;

    if (observation_info_send.resource != observation_info_received.resource)
      return false;

    if (observation_info_send.observation.size() != observation_info_received.observation.size())
      return false;

    for (size_t j = 0; j < observation_info_send.observation.size(); ++j)
    {
      tug_observers_msgs::observation observation_send = observation_info_send.observation[j];
      tug_observers_msgs::observation observation_received = observation_info_received.observation[j];

      if (observation_send.observation_msg != observation_received.observation_msg)
        return false;

      if (observation_send.verbose_observation_msg != observation_received.verbose_observation_msg)
        return false;

      if (observation_send.observation != observation_received.observation)
        return false;
    }
  }

  return true;
}

TEST_F(TestHelperObserverInfo, send_observer_info_test2)
{
  tug_observers_msgs::observer_info msg_to_send;

  std::pair<tug_observers_msgs::observer_info, bool> msg_pair = getMessage(
          boost::bind(&sendAndFlushInfoTestHelperFunction, msg_to_send), 1.1);
  EXPECT_FALSE(msg_pair.second);
}

TEST_F(TestHelperObserverInfo, send_observer_info_test3)
{
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_observer_info");
  return RUN_ALL_TESTS();
}
