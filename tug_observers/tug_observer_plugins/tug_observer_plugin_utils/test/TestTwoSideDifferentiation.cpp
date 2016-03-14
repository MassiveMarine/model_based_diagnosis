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
#include <tug_observer_plugin_utils/differentiation/TwoSideDifferentiation.h>

TEST(TwoSideDifferentiation, emtpy)
{
  TwoSideDifferentiation<double> differentation;
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, one_value1)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, one_value2)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, one_value3)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value1)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time::now());
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value2)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time::now());
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value3)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time::now());
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value4)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time::now());
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value5)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time::now());
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value6)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time::now());
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value7)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time::now());
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value8)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time::now());
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, two_value9)
{
  ros::Time::init();
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time::now());
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(TwoSideDifferentiation, differntation_test1)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test2)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test3)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test4)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test5)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test6)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test7)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test8)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test9)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test10)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test11)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(0., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test12)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test13)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(-2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test14)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(0., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test15)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(-2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test16)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(0., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test17)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test18)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test19)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(-2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test20)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test21)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test22)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test23)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test24)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-0., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test25)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test26)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test27)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-0., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test28)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test29)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test30)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test31)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test32)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test33)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test34)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test35)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test36)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test37)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test38)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test39)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test40)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test41)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test42)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test43)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test44)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test45)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test46)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(TwoSideDifferentiation, differntation_test47)
{
  TwoSideDifferentiation<double> differentation;
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
