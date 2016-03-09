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
#include <tug_observer_plugin_utils/filter/value_filter/MeanValueFilter.h>
#include <ros/ros.h>

TEST(MeanValueFilter, no_insert)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, one_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MeanValueFilter, one_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MeanValueFilter, one_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MeanValueFilter, reset_test1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, reset_test2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, reset_test3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert4)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert5)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert6)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert7)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert8)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, two_value_insert9)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert4)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert5)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert6)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert7)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, three_value_insert8)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert1)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert2)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert3)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert4)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert5)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert6)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert7)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert8)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert9)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert10)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert11)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert12)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert13)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert14)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert15)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, four_value_insert16)
{
  ros::param::set("/test/window_size", 3);
  XmlRpc::XmlRpcValue params;
  ros::param::get("/test", params);
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_no_insert)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_one_value_insert1)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_one_value_insert2)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_one_value_insert3)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_reset_test1)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_reset_test2)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_reset_test3)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert1)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert2)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert3)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert4)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert5)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert6)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert7)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert8)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_two_value_insert9)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert1)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert2)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert3)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert4)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert5)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert6)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert7)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_three_value_insert8)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert1)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert2)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(3. / 4., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert3)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert4)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert5)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert6)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. / 4., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert7)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert8)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert9)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert10)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1. / 4., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert11)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert12)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert13)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert14)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-3. / 4., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert15)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST(MeanValueFilter, no_buffer_four_value_insert16)
{
  XmlRpc::XmlRpcValue params;
  MeanValueFilter<double> filter(params);
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mean_filter");
  return RUN_ALL_TESTS();
}
