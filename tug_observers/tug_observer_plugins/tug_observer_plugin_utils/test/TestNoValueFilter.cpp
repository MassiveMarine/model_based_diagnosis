//
// Created by clemens on 18.11.15.
//

#include <gtest/gtest.h>
#include <tug_observer_plugin_utils/filter/value_filter/NoValueFilter.h>

TEST(NoValueFilter, no_insert)
{
  NoValueFilter<double> filter;
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, one_value_insert1)
{
  NoValueFilter<double> filter;
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, one_value_insert2)
{
  NoValueFilter<double> filter;
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, one_value_insert3)
{
  NoValueFilter<double> filter;
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, reset_test1)
{
  NoValueFilter<double> filter;
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, reset_test2)
{
  NoValueFilter<double> filter;
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, reset_test3)
{
  NoValueFilter<double> filter;
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert1)
{
  NoValueFilter<double> filter;
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert2)
{
  NoValueFilter<double> filter;
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert3)
{
  NoValueFilter<double> filter;
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert4)
{
  NoValueFilter<double> filter;
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert5)
{
  NoValueFilter<double> filter;
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert6)
{
  NoValueFilter<double> filter;
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert7)
{
  NoValueFilter<double> filter;
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert8)
{
  NoValueFilter<double> filter;
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(NoValueFilter, two_value_insert9)
{
  NoValueFilter<double> filter;
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
