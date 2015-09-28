//
// Created by clemens on 24.09.15.
//

#include <tug_velocity_observer/VelocityObserver.h>
#include <tug_yaml/ProcessYaml.h>
#include <tug_velocity_observer/VelocityConverterFactory.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

VelocityObserver::VelocityObserver(XmlRpc::XmlRpcValue params, SubscriberFacade* plugin_base) : use_roll_(false), use_pitch_(false),
                                       use_yaw_(false)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  if (!params.hasMember("source_A"))
  {
    ROS_ERROR("no topic for input A defined");
    throw std::invalid_argument("no topic for input A defined");
  }
  XmlRpc::XmlRpcValue source_a_params = params["source_A"];
  a_input_ = VelocityConverterFactory::createVelocityConverter(
          ProcessYaml::getValue<std::string>("type", source_a_params), source_a_params,
          boost::bind(&VelocityObserver::addTwistA, this, _1), plugin_base);

  if (!params.hasMember("source_B"))
  {
    ROS_ERROR("no topic for input B defined");
    throw std::invalid_argument("no topic for input B defined");
  }
  XmlRpc::XmlRpcValue source_b_params = params["source_B"];
  b_input_ = VelocityConverterFactory::createVelocityConverter(
          ProcessYaml::getValue<std::string>("type", source_b_params), source_b_params,
          boost::bind(&VelocityObserver::addTwistB, this, _1), plugin_base);

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

MovementReading VelocityObserver::getCompensatedTwist(MovementReading value)
{
  MovementReading result;
  if (x_filter_)
    result.linear.x = value.linear.x;
  else
    result.linear.x = 0.;

  if (y_filter_)
    result.linear.y = value.linear.y;
  else
    result.linear.y = 0.;

  if (z_filter_)
    result.linear.z = value.linear.z;
  else
    result.linear.z = 0.;

  if (use_roll_)
    result.angular.x = value.angular.x;
  else
    result.angular.x = 0.;

  if (use_pitch_)
    result.angular.y = value.angular.y;
  else
    result.angular.y = 0.;

  if (use_yaw_)
    result.angular.z = value.angular.z;
  else
    result.angular.z = 0.;

  return result;
}

void VelocityObserver::updateFilters(MovementReading a, MovementReading b)
{
  ROS_DEBUG("update filters");
  if (x_filter_)
    x_filter_->update(a.linear.x - b.linear.x);

  if (y_filter_)
    y_filter_->update(a.linear.y - b.linear.y);

  if (z_filter_)
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

  if (a.reading_time < b.reading_time)
    current_filter_time_ = b.reading_time;
  else
    current_filter_time_ = a.reading_time;

  ROS_DEBUG_STREAM("reading time sec:" << current_filter_time_.sec << ", nsec:" << current_filter_time_.nsec);
}

void VelocityObserver::addTwistA(MovementReading value)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM(
          "add twist of A with accelerations along the axis x:" << value.linear.x << " y:" << value.linear.y <<
          " z:" << value.linear.z << " velocities around the axis x:" << value.angular.x << " y:" <<
          value.angular.y << " z:" << value.angular.z);
  MovementReading compensated_a = getCompensatedTwist(value);
  a_twists_.push_back(compensated_a);

  while (!b_twists_.empty())
  {
    MovementReading b_twist = b_twists_.front();
    if (b_twist.reading_time > value.reading_time)
      break;

    b_twists_.pop_front();
    updateFilters(compensated_a, b_twist);
  }
}

void VelocityObserver::addTwistB(MovementReading value)
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM(
          "add twist of B with accelerations along the axis x:" << value.linear.x << " y:" << value.linear.y <<
          " z:" << value.linear.z << " velocities around the axis x:" << value.angular.x << " y:" <<
          value.angular.y << " z:" << value.angular.z);
  MovementReading compensated_b = getCompensatedTwist(value);
  b_twists_.push_back(compensated_b);

  while (!a_twists_.empty())
  {
    MovementReading a_twist = a_twists_.front();
    if (a_twist.reading_time > value.reading_time)
      break;

    a_twists_.pop_front();
    updateFilters(a_twist, compensated_b);
  }
}

std::pair<bool, std::vector<Observation> > VelocityObserver::estimateStates()
{
  boost::mutex::scoped_lock the_lock(filter_mutex_);

  ROS_DEBUG_STREAM("estimate states");
  FilteState<double> x_state;
  if (x_filter_)
  {
    x_state = x_filter_->getFilteState();
    ROS_DEBUG_STREAM("x filter result " << x_state);
    if (x_state.sample_size < 2)
      return std::make_pair(false, std::vector<Observation>());
  }

  FilteState<double> y_state;
  if (y_filter_)
  {
    y_state = y_filter_->getFilteState();
    ROS_DEBUG_STREAM("y filter result " << y_state);
    if (y_state.sample_size < 2)
      return std::make_pair(false, std::vector<Observation>());
  }

  FilteState<double> z_state;
  if (z_filter_)
  {
    z_state = z_filter_->getFilteState();
    ROS_DEBUG_STREAM("z filter result " << z_state);
    if (z_state.sample_size < 2)
      return std::make_pair(false, std::vector<Observation>());
  }

  FilteState<double> rot_x_state = rot_x_filter_->getFilteState();
  ROS_DEBUG_STREAM("rot x filter result " << rot_x_state);
  if (rot_x_state.sample_size < 2)
    return std::make_pair(false, std::vector<Observation>());

  FilteState<double> rot_y_state = rot_y_filter_->getFilteState();
  ROS_DEBUG_STREAM("rot y filter result " << rot_y_state);
  if (rot_y_state.sample_size < 2)
    return std::make_pair(false, std::vector<Observation>());

  FilteState<double> rot_z_state = rot_z_filter_->getFilteState();
  ROS_DEBUG_STREAM("rot z filter result " << rot_z_state);
  if (rot_z_state.sample_size < 2)
    return std::make_pair(false, std::vector<Observation>());

  ROS_DEBUG_STREAM("itterate through: " << states_.size() << " states");
  std::vector<Observation> result;
  for (std::vector<VelocityState>::iterator it = states_.begin(); it != states_.end(); ++it)
  {
    ROS_DEBUG_STREAM("check state: '" << it->getName() << "'");
    if (x_filter_ && !it->conformsStateX(x_state))
      continue;

    if (y_filter_ && !it->conformsStateY(y_state))
      continue;

    if (z_filter_ && !it->conformsStateZ(z_state))
      continue;

    if (it->conformsState(rot_x_state, rot_y_state, rot_z_state))
      result.push_back(Observation(it->getName(), it->getNumber()));
  }

  return std::make_pair(true, result);
}

ros::Time VelocityObserver::getCurrentFilterTime()
{
  return current_filter_time_;
}

std::string VelocityObserver::getName()
{
  return a_input_->getName() + "_" + b_input_->getName();
}