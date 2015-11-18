//
// Created by clemens on 17.11.15.
//

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}