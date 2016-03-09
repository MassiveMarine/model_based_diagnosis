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
