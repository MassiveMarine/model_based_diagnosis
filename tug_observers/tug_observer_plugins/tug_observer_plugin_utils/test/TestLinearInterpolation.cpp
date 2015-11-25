//
// Created by clemens on 18.11.15.
//

#include <gtest/gtest.h>
#include <tug_observer_plugin_utils/interpolation/LinearInterpolation.h>

TEST(LinearInterpolation, no_insert)
{
  LinearInterpolation<double> interpolation;
  EXPECT_FALSE(interpolation.hasNewInterpolatedPair());
}

TEST(LinearInterpolation, insert_A)
{
  LinearInterpolation<double> interpolation;
  ros::Time::init();
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time::now()));
  EXPECT_FALSE(interpolation.hasNewInterpolatedPair());
}

TEST(LinearInterpolation, insert_B)
{
  LinearInterpolation<double> interpolation;
  ros::Time::init();
  EXPECT_NO_THROW(interpolation.addFromB(0., ros::Time::now()));
  EXPECT_FALSE(interpolation.hasNewInterpolatedPair());
}

TEST(LinearInterpolation, insert_A_B)
{
  LinearInterpolation<double> interpolation;
  ros::Time::init();
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time::now()));
  EXPECT_NO_THROW(interpolation.addFromB(0., ros::Time::now()));
  EXPECT_FALSE(interpolation.hasNewInterpolatedPair());
}

TEST(LinearInterpolation, interpolate_A)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(1., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromA(2., ros::Time(2.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_B)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromB(0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromB(2., ros::Time(2.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_A2)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(1., ros::Time(2.)));
  EXPECT_NO_THROW(interpolation.addFromA(2., ros::Time(4.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_A3)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromB(1., ros::Time(3.)));
  EXPECT_NO_THROW(interpolation.addFromA(2., ros::Time(5.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_A4)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(-0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(-1., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromA(-2., ros::Time(2.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(-1., result.first);
  EXPECT_FLOAT_EQ(-1., result.second);
}

TEST(LinearInterpolation, interpolate_A5)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(-0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(-1., ros::Time(2.)));
  EXPECT_NO_THROW(interpolation.addFromA(-2., ros::Time(4.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(-1., result.first);
  EXPECT_FLOAT_EQ(-1., result.second);
}

TEST(LinearInterpolation, interpolate_A6)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(-0., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromB(-1., ros::Time(3.)));
  EXPECT_NO_THROW(interpolation.addFromA(-2., ros::Time(5.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(-1., result.first);
  EXPECT_FLOAT_EQ(-1., result.second);
}

TEST(LinearInterpolation, interpolate_A7)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(1., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(2.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_A8)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(1., ros::Time(2.)));
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(4.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_A9)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromB(1., ros::Time(3.)));
  EXPECT_NO_THROW(interpolation.addFromA(1., ros::Time(5.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(1., result.first);
  EXPECT_FLOAT_EQ(1., result.second);
}

TEST(LinearInterpolation, interpolate_A10)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(0., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(2.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(0., result.first);
  EXPECT_FLOAT_EQ(0., result.second);
}

TEST(LinearInterpolation, interpolate_A11)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(0.)));
  EXPECT_NO_THROW(interpolation.addFromB(0., ros::Time(2.)));
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(4.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(0., result.first);
  EXPECT_FLOAT_EQ(0., result.second);
}

TEST(LinearInterpolation, interpolate_A12)
{
  LinearInterpolation<double> interpolation;
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(1.)));
  EXPECT_NO_THROW(interpolation.addFromB(0., ros::Time(3.)));
  EXPECT_NO_THROW(interpolation.addFromA(0., ros::Time(5.)));
  EXPECT_TRUE(interpolation.hasNewInterpolatedPair());
  std::pair<double, double> result = interpolation.getNextInterpolatedPair();
  EXPECT_FLOAT_EQ(0., result.first);
  EXPECT_FLOAT_EQ(0., result.second);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}