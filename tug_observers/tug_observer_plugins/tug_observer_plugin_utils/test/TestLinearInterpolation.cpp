/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <tug_observer_plugin_utils/interpolation/LinearInterpolation.h>
#include <utility>

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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
