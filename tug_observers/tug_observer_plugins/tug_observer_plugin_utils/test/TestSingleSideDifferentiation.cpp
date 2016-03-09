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
#include <tug_observer_plugin_utils/differentiation/SingleSideDifferentiation.h>

TEST(SingleSideDifferentiation, emtpy)
{
  SingleSideDifferentiation<double> differentation;
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SingleSideDifferentiation, one_value1)
{
  ros::Time::init();
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SingleSideDifferentiation, one_value2)
{
  ros::Time::init();
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SingleSideDifferentiation, one_value3)
{
  ros::Time::init();
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SingleSideDifferentiation, differntation_test1)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test2)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test3)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test4)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test5)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test6)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test7)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test8)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test9)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test10)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test11)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test12)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SingleSideDifferentiation, differntation_test13)
{
  SingleSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.).toSec(), differentation.getDifferntiationTime().toSec());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
