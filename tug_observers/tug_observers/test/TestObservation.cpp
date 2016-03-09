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
#include <tug_observers/Observation.h>

TEST(Observation, constructor_test_1)
{
  Observation observation("", 0);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, 0);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_2)
{
  Observation observation("", -1);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, -1);
  EXPECT_TRUE(observation.isFaulty());
}

TEST(Observation, constructor_test_3)
{
  Observation observation("", 2);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, 2);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_4)
{
  Observation observation("a", 0);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, 0);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_5)
{
  Observation observation("a", -1);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, -1);
  EXPECT_TRUE(observation.isFaulty());
}

TEST(Observation, constructor_test_6)
{
  Observation observation("a", 2);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, 2);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_7)
{
  Observation observation("", "", 0);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, 0);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_8)
{
  Observation observation("", "", -1);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, -1);
  EXPECT_TRUE(observation.isFaulty());
}

TEST(Observation, constructor_test_9)
{
  Observation observation("", "", 2);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, 2);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_10)
{
  Observation observation("a", "", 0);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, 0);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_11)
{
  Observation observation("a", "", -1);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, -1);
  EXPECT_TRUE(observation.isFaulty());
}

TEST(Observation, constructor_test_12)
{
  Observation observation("a", "", 2);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "");
  EXPECT_EQ(as_msg.observation, 2);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_13)
{
  Observation observation("", "a", 0);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, 0);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_14)
{
  Observation observation("", "a", -1);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, -1);
  EXPECT_TRUE(observation.isFaulty());
}

TEST(Observation, constructor_test_15)
{
  Observation observation("", "a", 2);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, 2);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_16)
{
  Observation observation("a", "a", 0);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, 0);
  EXPECT_FALSE(observation.isFaulty());
}

TEST(Observation, constructor_test_17)
{
  Observation observation("a", "a", -1);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, -1);
  EXPECT_TRUE(observation.isFaulty());
}

TEST(Observation, constructor_test_18)
{
  Observation observation("a", "a", 2);

  tug_observers_msgs::observation as_msg = observation.toMsg();
  EXPECT_EQ(as_msg.observation_msg, "a");
  EXPECT_EQ(as_msg.verbose_observation_msg, "a");
  EXPECT_EQ(as_msg.observation, 2);
  EXPECT_FALSE(observation.isFaulty());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
