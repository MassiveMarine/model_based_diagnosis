//
// Created by clemens on 22.09.15.
//

#include <tug_hz_observer/HzBase.h>

HzBase::HzBase(XmlRpc::XmlRpcValue params)
{
  if (!params.hasMember("filter"))
  {
    ROS_ERROR("No filter for hz plugin defined");
    throw std::runtime_error("No filter for hz plugin defined");
  }
  XmlRpc::XmlRpcValue filter_params = params["filter"];
  time_filter_ = boost::make_shared<Filter<double> >(filter_params);
}

void HzBase::update()
{
  ros::Time time_now = ros::Time::now();
  if(has_last_messurment_)
  {
    time_filter_->update((time_now - last_time_).toSec());
  }

  last_time_ = time_now;
  has_last_messurment_ = true;
}

FilteState<double> HzBase::getFilterState()
{
  return time_filter_->getFilteState();
}
