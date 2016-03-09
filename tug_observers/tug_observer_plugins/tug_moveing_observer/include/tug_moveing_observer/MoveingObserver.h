/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_MOVEING_OBSERVER_MOVEINGOBSERVER_H
#define TUG_MOVEING_OBSERVER_MOVEINGOBSERVER_H

#include <geometry_msgs/TwistStamped.h>
#include <list>
#include <tug_observer_plugin_utils/filter/Filter.h>
#include <tug_moveing_observer/VelocityState.h>
#include <tug_moveing_observer/VelocityConverter.h>
#include <tug_moveing_observer/MovementReading.h>
#include <tug_observers/SubscriberFacade.h>
#include <tug_observers/Observation.h>
#include <string>
#include <utility>
#include <vector>

class MoveingObserver
{
    boost::shared_ptr<VelocityConverter> input_;

    boost::shared_ptr<Filter<double> > x_filter_;
    boost::shared_ptr<Filter<double> > y_filter_;
    boost::shared_ptr<Filter<double> > z_filter_;

    bool use_roll_;
    bool use_pitch_;
    bool use_yaw_;
    boost::shared_ptr<Filter<double> > rot_x_filter_;
    boost::shared_ptr<Filter<double> > rot_y_filter_;
    boost::shared_ptr<Filter<double> > rot_z_filter_;

    std::vector<VelocityState> states_;

    ros::Time current_filter_time_;

    boost::mutex filter_mutex_;

    MovementReading getCompensatedTwist(MovementReading value);

    void updateFilters(MovementReading a);

    ros::NodeHandle nh_;
    ros::Publisher a_publisher_;
    ros::AsyncSpinner spinner_;

public:
    MoveingObserver(XmlRpc::XmlRpcValue params, SubscriberFacade *plugin_base);

    void addTwist(MovementReading value);

    std::pair<bool, std::vector<Observation> > estimateStates();

    ros::Time getCurrentFilterTime();

    std::string getName();
};


#endif  // TUG_MOVEING_OBSERVER_MOVEINGOBSERVER_H
