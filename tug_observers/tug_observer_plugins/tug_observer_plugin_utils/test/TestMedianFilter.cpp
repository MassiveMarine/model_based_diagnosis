//
// Created by clemens on 18.11.15.
//

#include <gtest/gtest.h>
#include <tug_observer_plugin_utils/filter/value_filter/MedianValueFilter.h>
#include <ros/ros.h>

TEST(MedianValueFilter, no_insert)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MedianValueFilter, one_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MedianValueFilter, one_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MedianValueFilter, one_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MedianValueFilter, reset_test1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MedianValueFilter, reset_test2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MedianValueFilter, reset_test3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert4)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert5)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert6)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert7)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert8)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, two_value_insert9)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert4)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert5)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert6)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert7)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, three_value_insert8)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert4)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert5)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert6)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert7)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert8)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert9)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert10)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert11)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert12)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert13)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert14)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert15)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MedianValueFilter, four_value_insert16)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MedianValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_median_filter");
  return RUN_ALL_TESTS();
}
