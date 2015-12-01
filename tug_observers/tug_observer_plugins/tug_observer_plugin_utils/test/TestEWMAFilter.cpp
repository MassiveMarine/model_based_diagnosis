//
// Created by clemens on 18.11.15.
//

#include <gtest/gtest.h>
#include <tug_observer_plugin_utils/filter/value_filter/EWMAValueFilter.h>
#include <ros/ros.h>
#include <tug_testing/ParameterHelper.h>

class EWMAValueFilterHelper : public ::testing::Test
{
protected:
    ParameterHelper param_helper_;
};

TEST_F(EWMAValueFilterHelper, no_insert)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, one_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, one_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, one_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, reset_test1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, reset_test2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, reset_test3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. * 0.8, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1. * 0.8, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1. * 0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1. * 0.8 + -1. * 0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1. * 0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. * 0.8 + 1. * 0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, two_value_insert9)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.8 + 2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, three_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.8 + -2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.8 + 2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.8 + -2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert9)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert10)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert11)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert12)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.8 + 2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert13)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert14)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert15)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, four_value_insert16)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.8 + -2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_no_insert)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_one_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_one_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_one_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_reset_test1)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_reset_test2)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_reset_test3)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-0.8, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0.8, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert4)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert5)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert6)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8 + -1.*0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert7)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert8)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8 + 1.*0.2, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_two_value_insert9)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.8 + (1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert4)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.8 + 2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert5)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert6)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert7)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.8 + (-1.*0.8)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_three_value_insert8)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.8 + -2.*0.2)*0.2, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8+(1.*0.8+(0.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.8+(1.*0.8+(0.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert4)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.8+(1.*0.8+(2.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert5)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8+(-1.*0.8+(-1.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert6)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8+(-1.*0.8+(0.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert7)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.8+(-1.*0.8+(0.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert8)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.8+(-1.*0.8+(-2.0*0.8+(1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert9)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8+(1.*0.8+(1.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert10)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.8+(1.*0.8+(0.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert11)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.8+(1.*0.8+(0.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert12)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.8+(1.*0.8+(2.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert13)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert14)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.8+(-1.*0.8+(0.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert15)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.8+(-1.*0.8+(0.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, no_buffer_four_value_insert16)
{
  param_helper_.setTmpParameter("decay_rate", 0.8);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.8+(-1.*0.8+(-2.0*0.8+(-1.0*0.2))*0.2)*0.2, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_insert)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_one_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_one_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_one_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_reset_test1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_reset_test2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_reset_test3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. * 0.3, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1. * 0.3, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1. * 0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1. * 0.3 + -1. * 0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1. * 0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. * 0.3 + 1. * 0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_two_value_insert9)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.3 + 2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_three_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.3 + -2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.3 + 2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.3 + -2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert9)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert10)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert11)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert12)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.3 + 2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert13)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert14)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert15)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_four_value_insert16)
{
  param_helper_.setTmpParameter("window_size", 3);
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.3 + -2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_no_insert)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_one_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_one_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_one_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_reset_test1)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_reset_test2)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_reset_test3)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-0.3, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0.3, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert4)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert5)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert6)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3 + -1.*0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert7)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert8)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3 + 1.*0.7, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_two_value_insert9)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.3 + (1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert4)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((1.*0.3 + 2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert5)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert6)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert7)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.3 + (-1.*0.3)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_three_value_insert8)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ((-1.*0.3 + -2.*0.7)*0.7, filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert1)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert2)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3+(1.*0.3+(0.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert3)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.3+(1.*0.3+(0.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert4)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.3+(1.*0.3+(2.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert5)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3+(-1.*0.3+(-1.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert6)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3+(-1.*0.3+(0.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert7)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.3+(-1.*0.3+(0.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert8)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.3+(-1.*0.3+(-2.0*0.3+(1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert9)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3+(1.*0.3+(1.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert10)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1.*0.3+(1.*0.3+(0.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert11)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(2.*0.3+(1.*0.3+(0.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert12)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.3+(1.*0.3+(2.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert13)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert14)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1.*0.3+(-1.*0.3+(0.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert15)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2.*0.3+(-1.*0.3+(0.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(EWMAValueFilterHelper, small_no_buffer_four_value_insert16)
{
  param_helper_.setTmpParameter("decay_rate", 0.3);
  EWMAValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.*0.3+(-1.*0.3+(-2.0*0.3+(-1.0*0.7))*0.7)*0.7, filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mean_filter");
  return RUN_ALL_TESTS();
}
