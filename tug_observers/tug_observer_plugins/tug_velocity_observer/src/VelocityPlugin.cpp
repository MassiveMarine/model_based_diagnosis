//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityPlugin.h>
#include <pluginlib/class_list_macros.h>
#include <tug_observer_plugin_utils/ProcessYaml.h>
#include <tug_velocity_observer/VelocityConverterFactory.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

VelocityPlugin::VelocityPlugin() : ObserverPluginBase("velocity"), use_roll_(false), use_pitch_(false), use_yaw_(false)
{ }

void VelocityPlugin::initialize(XmlRpc::XmlRpcValue params)
{
  if(!params.hasMember("topic_A"))
  {
    ROS_ERROR("no topic for input A defined");
    throw std::invalid_argument("no topic for input A defined");
  }
  XmlRpc::XmlRpcValue topic_a_params = params["topic_A"];
  a_input_ = VelocityConverterFactory::createVelocityConverter(ProcessYaml::getValue<std::string>("type", topic_a_params), topic_a_params, boost::bind(&VelocityPlugin::addTwistA, this, _1), this);

  if(!params.hasMember("topic_B"))
  {
    ROS_ERROR("no topic for input B defined");
    throw std::invalid_argument("no topic for input B defined");
  }
  XmlRpc::XmlRpcValue topic_b_params = params["topic_B"];
  b_input_ = VelocityConverterFactory::createVelocityConverter(ProcessYaml::getValue<std::string>("type", topic_b_params), topic_b_params, boost::bind(&VelocityPlugin::addTwistB, this, _1), this);

  if (params.hasMember("x_filter"))
  {
    XmlRpc::XmlRpcValue x_filter_params = params["x_filter"];
    x_filter_ = boost::make_shared<Filter<double> >(x_filter_params);
  }

  if (params.hasMember("y_filter"))
  {
    XmlRpc::XmlRpcValue y_filter_params = params["y_filter"];
    y_filter_ = boost::make_shared<Filter<double> >(y_filter_params);
  }

  if (params.hasMember("z_filter"))
  {
    XmlRpc::XmlRpcValue z_filter_params = params["z_filter"];
    z_filter_ = boost::make_shared<Filter<double> >(z_filter_params);
  }

  use_roll_ = ProcessYaml::getValue<bool>("use_roll", params, false);
  use_pitch_ = ProcessYaml::getValue<bool>("use_pitch", params, false);
  use_yaw_ = ProcessYaml::getValue<bool>("use_yaw", params, false);

  if (!params.hasMember("rot_x_filter"))
  {
    ROS_ERROR("No rot_x_filter for velocity plugin defined");
    throw std::runtime_error("No rot_x_filter for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue rot_x_filter_params = params["rot_x_filter"];
  rot_x_filter_ = boost::make_shared<Filter<double> >(rot_x_filter_params);

  if (!params.hasMember("rot_y_filter"))
  {
    ROS_ERROR("No rot_y_filter for velocity plugin defined");
    throw std::runtime_error("No rot_y_filter for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue rot_y_filter_params = params["rot_y_filter"];
  rot_y_filter_ = boost::make_shared<Filter<double> >(rot_y_filter_params);

  if (!params.hasMember("rot_z_filter"))
  {
    ROS_ERROR("No rot_z_filter for velocity plugin defined");
    throw std::runtime_error("No rot_z_filter for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue rot_z_filter_params = params["rot_z_filter"];
  rot_z_filter_ = boost::make_shared<Filter<double> >(rot_z_filter_params);

  if (!params.hasMember("states"))
  {
    ROS_ERROR("No states for velocity plugin defined");
    throw std::runtime_error("No states for velocity plugin defined");
  }
  XmlRpc::XmlRpcValue state_params = params["states"];
  for (int i = 0; i < state_params.size(); ++i)
    states_.push_back(VelocityState(state_params[i]));
}

MovementReading VelocityPlugin::getCompensatedTwist(MovementReading value)
{
  MovementReading result;
  if(x_filter_)
    result.linear.x = value.linear.x;
  else
    result.linear.x = 0.;

  if(y_filter_)
    result.linear.y = value.linear.y;
  else
    result.linear.y = 0.;

  if(z_filter_)
    result.linear.z = value.linear.z;
  else
    result.linear.z = 0.;

  if(use_roll_)
    result.angular.x = value.angular.x;
  else
    result.angular.x = 0.;

  if(use_pitch_)
    result.angular.y = value.angular.y;
  else
    result.angular.y = 0.;

  if(use_yaw_)
    result.angular.z = value.angular.z;
  else
    result.angular.z = 0.;
}

void VelocityPlugin::updateFilters(MovementReading a, MovementReading b)
{
  if(x_filter_)
    x_filter_->update(a.linear.x - b.linear.x);

  if(y_filter_)
    y_filter_->update(a.linear.y - b.linear.y);

  if(z_filter_)
    z_filter_->update(a.linear.z - b.linear.z);

  tf::Quaternion rotation_quaternion_a;
  rotation_quaternion_a.setRPY(a.angular.x, a.angular.y, a.angular.z);
  rotation_quaternion_a.normalize();

  tf::Quaternion rotation_quaternion_b;
  rotation_quaternion_b.setRPY(b.angular.x, b.angular.y, b.angular.z);
  rotation_quaternion_b.normalize();

  tf::Quaternion shortest_difference = rotation_quaternion_a.nearest(rotation_quaternion_b);
  rot_x_filter_->update(shortest_difference.x());
  rot_y_filter_->update(shortest_difference.y());
  rot_z_filter_->update(shortest_difference.z());
}

void VelocityPlugin::addTwistA(MovementReading value)
{
  MovementReading compensated_a = getCompensatedTwist(value);
  a_twists_.push_back(compensated_a);

  while(!b_twists_.empty())
  {
    MovementReading b_twist = b_twists_.front();
    if(b_twist.reading_time > value.reading_time)
      break;

    b_twists_.pop_front();
    updateFilters(compensated_a, b_twist);
  }
}

void VelocityPlugin::addTwistB(MovementReading value)
{
  MovementReading compensated_b = getCompensatedTwist(value);
  b_twists_.push_back(compensated_b);

  while(!a_twists_.empty())
  {
    MovementReading a_twist = a_twists_.front();
    if(a_twist.reading_time > value.reading_time)
      break;

    a_twists_.pop_front();
    updateFilters(a_twist, compensated_b);
  }
}

std::vector<std::string> VelocityPlugin::estimateStates()
{
  FilteState<double> x_state;
  if(x_filter_)
    x_state = x_filter_->getFilteState();

  FilteState<double> y_state;
  if(y_filter_)
    y_state = y_filter_->getFilteState();

  FilteState<double> z_state;
  if(z_filter_)
    z_state = z_filter_->getFilteState();

  FilteState<double> rot_x_state = rot_x_filter_->getFilteState();
  FilteState<double> rot_y_state = rot_y_filter_->getFilteState();
  FilteState<double> rot_z_state = rot_z_filter_->getFilteState();

  std::vector<std::string> result;
  for(std::vector<VelocityState>::iterator it = states_.begin(); it != states_.end(); ++it)
  {
    if(x_filter_ && !it->conformsStateX(x_state))
      continue;

    if(y_filter_ && !it->conformsStateY(y_state))
      continue;

    if(z_filter_ && !it->conformsStateZ(z_state))
      continue;

    if(it->conformsState(rot_x_state, rot_y_state, rot_z_state))
      result.push_back(it->getName());
  }

  return result;
}

PLUGINLIB_EXPORT_CLASS(VelocityPlugin, tug_observers::ObserverPluginBase)
