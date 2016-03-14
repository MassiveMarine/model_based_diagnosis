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
#include <tug_observer_plugin_utils/differentiation/SimpleLinearRegression.h>

TEST(SimpleLinearRegression, wrong_constructor1)
{
  EXPECT_ANY_THROW(SimpleLinearRegression<double> differentation(0));
}

TEST(SimpleLinearRegression, wrong_constructor2)
{
  EXPECT_ANY_THROW(SimpleLinearRegression<double> differentation(1));
}

TEST(SimpleLinearRegression, emtpy)
{
  SimpleLinearRegression<double> differentation(3);
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, one_value1)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, one_value2)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, one_value3)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value1)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time::now());
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value2)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time::now());
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value3)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time::now());
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value4)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time::now());
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value5)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time::now());
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value6)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time::now());
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value7)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time::now());
  differentation.addValue(0., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value8)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time::now());
  differentation.addValue(-2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, two_value9)
{
  ros::Time::init();
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time::now());
  differentation.addValue(2., ros::Time::now());
  EXPECT_FALSE(differentation.hasDifferentiation());
}

TEST(SimpleLinearRegression, differntation_test1)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test2)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test3)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test4)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test5)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test6)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test7)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test8)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test9)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test10)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test11)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(0., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test12)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test13)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(-2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test14)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(0., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test15)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  differentation.addValue(-2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test16)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(0., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test17)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test18)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test19)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(-2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test20)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test21)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test22)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test23)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test24)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-0., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test25)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test26)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test27)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-0., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test28)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test29)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test30)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test31)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test32)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test33)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test34)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-1., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test35)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test36)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test37)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-1., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test38)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test39)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test40)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(3., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test41)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(2., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test42)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test43)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(4., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test44)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  differentation.addValue(-1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test45)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(6., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test46)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(4., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test47)
{
  SimpleLinearRegression<double> differentation(3);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  differentation.addValue(-1., ros::Time(4.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(2.).toSec(), differentation.getDifferntiationTime().toSec());
}


TEST(SimpleLinearRegression, differntation_test48)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.5).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test49)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.5).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test50)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test51)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(1., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test52)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-1., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test53)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.5).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test54)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(1.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(1., differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(0.5).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test55)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(2.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test56)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(3., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test57)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-3., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.5, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test58)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(0., ros::Time(0.));
  differentation.addValue(0., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test59)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(2., ros::Time(0.));
  differentation.addValue(2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

TEST(SimpleLinearRegression, differntation_test60)
{
  SimpleLinearRegression<double> differentation(2);
  differentation.addValue(-2., ros::Time(0.));
  differentation.addValue(-2., ros::Time(2.));
  EXPECT_TRUE(differentation.hasDifferentiation());
  EXPECT_FLOAT_EQ(0.0, differentation.getDifferentiation());
  EXPECT_DOUBLE_EQ(ros::Time(1.).toSec(), differentation.getDifferntiationTime().toSec());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
