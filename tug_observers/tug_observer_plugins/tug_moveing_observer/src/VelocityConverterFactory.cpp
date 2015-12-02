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

#include <tug_moveing_observer/VelocityConverterFactory.h>
#include <tug_moveing_observer/VelocityConverterTwist.h>
#include <tug_moveing_observer/VelocityConverterTwistStamped.h>
#include <tug_moveing_observer/VelocityConverterIMU.h>
#include <tug_moveing_observer/VelocityConverterOdometry.h>
#include <tug_moveing_observer/VelocityConverterPoseStamped.h>
#include <tug_moveing_observer/VelocityConverterTf.h>
#include <stdexcept>
#include <string>

boost::shared_ptr<VelocityConverter> VelocityConverterFactory::createVelocityConverter(std::string type,
                                                                     XmlRpc::XmlRpcValue params,
                                                                     boost::function<void(MovementReading)> call_back,
                                                                     SubscriberFacade *plugin_base)
{
  if (type == "twist")
    return boost::make_shared<VelocityConverterTwist>(params, call_back, plugin_base);
  else if (type == "twist_stamped")
    return boost::make_shared<VelocityConverterTwistStamped>(params, call_back, plugin_base);
  else if (type == "imu")
    return boost::make_shared<VelocityConverterIMU>(params, call_back, plugin_base);
  else if (type == "odometry")
    return boost::make_shared<VelocityConverterOdometry>(params, call_back, plugin_base);
  else if (type == "pose_stamped")
    return boost::make_shared<VelocityConverterPoseStamped>(params, call_back, plugin_base);
  else if (type == "tf")
    return boost::make_shared<VelocityConverterTf>(params, call_back);
  else
    throw std::runtime_error("type for nominal value '" + type + "'" + " not known");
}
