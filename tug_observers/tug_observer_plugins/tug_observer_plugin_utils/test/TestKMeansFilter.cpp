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
#include <tug_observer_plugin_utils/filter/value_filter/KMeansValueFilter.h>
#include <ros/ros.h>
#include <tug_testing/ParameterHelper.h>

class KMeansFilterHelper : public ::testing::Test
{
protected:
    ParameterHelper param_helper_;
};

TEST_F(KMeansFilterHelper, no_insert)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, one_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, one_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, one_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(1, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, reset_test1)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, reset_test2)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, reset_test3)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  filter.reset();
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(0, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0.5, filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, two_value_insert9)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(2, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, three_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(3, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert1)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert2)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert3)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(4. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert4)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(4. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert5)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert6)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert7)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert8)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(0., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert9)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(1.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert10)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(1.);
  EXPECT_FLOAT_EQ(2. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert11)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(1.);
  filter.update(2.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert12)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(2.);
  filter.update(1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert13)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-1., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert14)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-1.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert15)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(0.);
  filter.update(-1.);
  filter.update(-2.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, four_value_insert16)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(-1.);
  filter.update(-2.);
  filter.update(-1.);
  filter.update(0.);
  EXPECT_FLOAT_EQ(-2. / 3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, odd_odd_test)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(3.);
  filter.update(4.);
  filter.update(5.);
  EXPECT_FLOAT_EQ(3., filter.getValue());
  EXPECT_EQ(5, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, even_odd_test)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 2);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(3.);
  filter.update(4.);
  filter.update(5.);
  EXPECT_FLOAT_EQ(7. / 2., filter.getValue());
  EXPECT_EQ(5, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, odd_even_test)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 3);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(3.);
  filter.update(4.);
  EXPECT_FLOAT_EQ(3., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

TEST_F(KMeansFilterHelper, even_even_test)
{
  param_helper_.setTmpParameter("window_size", 5);
  param_helper_.setTmpParameter("k_size", 2);
  KMeansValueFilter<double> filter(param_helper_.getTmpParams());
  filter.update(1.);
  filter.update(2.);
  filter.update(3.);
  filter.update(4.);
  EXPECT_FLOAT_EQ(5. / 2., filter.getValue());
  EXPECT_EQ(4, filter.getSampleSize());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_median_filter");
  return RUN_ALL_TESTS();
}
