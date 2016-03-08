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

#include <tug_velocity_observer/VelocityChecker.h>
#include <string>
#include <boost/tuple/tuple.hpp>
#include <map>
#include <utility>
#include <vector>
#include <set>

VelocityChecker::VelocityChecker()
{ }

void VelocityChecker::init(XmlRpc::XmlRpcValue params, SubscriberFacade *plugin_base)
{
  ROS_DEBUG("VelocityChecker::init 1");
  if (!params.hasMember("correlations"))
  {
    ROS_ERROR("No correlations for velocity plugin defined");
    throw std::runtime_error("No correlations for velocity plugin defined");
  }
  ROS_DEBUG("VelocityChecker::init 2");
  XmlRpc::XmlRpcValue correlations_params = params["correlations"];
  for (int i = 0; i < correlations_params.size(); ++i)
    observers_.push_back(boost::make_shared<VelocityObserver>(correlations_params[i], plugin_base));
  ROS_DEBUG("VelocityChecker::init 3");
}

std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > VelocityChecker::velocityObservations()
{
  std::vector<boost::tuple<std::string, std::vector<Observation>, ros::Time> > result;
  for (std::vector<boost::shared_ptr<VelocityObserver> >::iterator it = observers_.begin();
       it != observers_.end(); ++it)
  {
    std::pair<bool, std::vector<Observation> > states = (*it)->estimateStates();
    if (states.first)
    {
      ROS_DEBUG_STREAM("got estimated states: " << states.second.size());
      result.push_back(
              boost::make_tuple<std::string, std::vector<Observation>, ros::Time>((*it)->getName(), states.second,
                                                                                  (*it)->getCurrentFilterTime()));
    }
    else
    {
      result.push_back(
              boost::make_tuple<std::string, std::vector<Observation>, ros::Time>((*it)->getName(),
                                                                                  std::vector<Observation>(),
                                                                                  (*it)->getCurrentFilterTime()));
      ROS_DEBUG("got no estimated states");
    }
  }

  return result;
}

std::map<std::string, bool> VelocityChecker::getValidInputs()
{
  // we use a simple counting based diagnosis to decide which component is faulty. As long as no correlation or common
  // inputs are considered between inputs to the velocity checker this is a simple and valid method.
  std::map<std::string, unsigned int> fault_count;
  std::set<std::string> inputs;

  // flag to optimze case of no fault
  bool has_fault = false;
  // increment the count for each input if a faulty state is plausible
  for (std::vector<boost::shared_ptr<VelocityObserver> >::iterator it = observers_.begin();
       it != observers_.end(); ++it)
  {
    std::pair<bool, std::vector<Observation> > states = (*it)->estimateStates();
    if (states.first)
    {
      std::string input_a = (*it)->getInputAName();
      inputs.insert(input_a);
      std::string input_b = (*it)->getInputBName();
      inputs.insert(input_b);

      for (size_t i = 0; i < states.second.size(); ++i)
      {
        if (states.second[i].isFaulty())
        {
          has_fault = true;
          updateFaultCount(input_a, 1, &fault_count);
          updateFaultCount(input_b, 1, &fault_count);
        }
      }
    }
    else
      ROS_DEBUG("got no estimated states");
  }

  std::map<std::string, bool> result;
  // enshure every input has a count
  for (std::set<std::string>::iterator it = inputs.begin(); it != inputs.end(); ++it)
  {
    updateFaultCount(*it, 0, &fault_count);
    result.insert(std::make_pair(*it, true));
  }

  if (has_fault)
  {
    // get maximum count -> shows faulty input
    unsigned int maximum = 0;
    for (std::map<std::string, unsigned int>::iterator it = fault_count.begin(); it != fault_count.end(); ++it)
    {
      if (maximum < it->second)
        maximum = it->second;
    }
    // all inputs with the maximum count are faulty
    for (std::map<std::string, unsigned int>::iterator it = fault_count.begin(); it != fault_count.end(); ++it)
    {
      if (it->second == maximum)
        result[it->first] = false;
    }
  }

  return result;
}

void VelocityChecker::updateFaultCount(std::string name, unsigned int increment,
                                       std::map<std::string, unsigned int> *fault_count)
{
  std::map<std::string, unsigned int>::iterator name_pos = fault_count->find(name);
  if (name_pos != fault_count->end())
    name_pos->second += increment;
  else
  {
    fault_count->insert(std::make_pair(name, increment));
  }
}
