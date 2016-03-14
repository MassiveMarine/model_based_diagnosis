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

#ifndef TUG_OBSERVER_PLUGIN_UTILS_INTEGRATION_SIMPLESUMMBASEDINTEGRATION_H
#define TUG_OBSERVER_PLUGIN_UTILS_INTEGRATION_SIMPLESUMMBASEDINTEGRATION_H

#include <tug_observer_plugin_utils/integration/Integration.h>

template<class T>
class SimpleSummBasedIntegration : public Integration<T>
{
    T past_value_;
    ros::Time past_value_time_;
    bool has_past_value_;
    T current_value_;

public:
    SimpleSummBasedIntegration() : has_past_value_(false)
    { }

    virtual void addValue(const T &value, const ros::Time &value_time)
    {
      if (has_past_value_)
      {
        if (value_time == past_value_time_)
        {
          ROS_DEBUG_STREAM("skip value for calculation integration with new value:" << value <<
                           " current time sec:" << value_time.sec <<
                           " nsec:" << value_time.nsec << " old time sec: " << past_value_time_.sec <<
                           " nsec:" << past_value_time_.nsec);
          return;
        }
        else if (value_time < past_value_time_)
        {
          ROS_ERROR_STREAM("can't calculate integration with new value:" << value <<
                           " current time sec:" << value_time.sec <<
                           " nsec:" << value_time.nsec << " old time sec: " << past_value_time_.sec <<
                           " nsec:" << past_value_time_.nsec);
          throw std::invalid_argument("new added value is in the past can't process this data");
        }

        double time_difference = (value_time - past_value_time_).toSec();

        current_value_ += static_cast<T>(static_cast<double>(past_value_) * time_difference);
      }

      past_value_time_ = value_time;
      past_value_ = value;
      has_past_value_ = true;
    }

    virtual bool hasIntegration()
    {
      return has_past_value_;
    }

    virtual T getIntegration()
    {
      return current_value_;
    }

    virtual ros::Time getIntegrationTime()
    {
      return past_value_time_;
    }
};


#endif  // TUG_OBSERVER_PLUGIN_UTILS_INTEGRATION_SIMPLESUMMBASEDINTEGRATION_H
