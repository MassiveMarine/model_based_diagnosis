//
// Created by clemens on 18.11.15.
//

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}