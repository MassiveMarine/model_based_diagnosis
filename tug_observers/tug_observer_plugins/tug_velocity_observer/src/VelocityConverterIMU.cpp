//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterIMU.h>
#include <tug_yaml/ProcessYaml.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

VelocityConverterIMU::VelocityConverterIMU(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, SubscriberFacade* plugin_base) : VelocityConverter(params, call_back)
{
  topic_ = ProcessYaml::getValue<std::string>("topic", params);
  imu_sub_ = plugin_base->subscribe(topic_, 1, &VelocityConverterIMU::IMUCB, this);
  graviation_cancelation_ = ProcessYaml::getValue<bool>("gravitation_cancelation", params, false);
}

void VelocityConverterIMU::IMUCB(const sensor_msgs::Imu& msg)
{
  MovementReading reading;
  reading.reading_time = msg.header.stamp;
  if(!graviation_cancelation_)
  {
    reading.linear.x = msg.linear_acceleration.x;
    reading.linear.y = msg.linear_acceleration.y;
    reading.linear.z = msg.linear_acceleration.z;
  }
  else
  {
    // gravity cancelation following
    // http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus
 /*   # compensate the accelerometer readings from gravity.
    # @param q the quaternion representing the orientation of a 9DOM MARG sensor array
    # @param acc the readings coming from an accelerometer expressed in g
    #
    # @return a 3d vector representing dinamic acceleration expressed in g
    def gravity_compensate(q, acc):
      g = [0.0, 0.0, 0.0]

      # get expected direction of gravity
      g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
      g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
      g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]

      # compensate accelerometer readings with the expected direction of gravity
      return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]*/

    double g_x = 2. * (msg.orientation.x * msg.orientation.z - msg.orientation.w * msg.orientation.y);
    double g_y = 2. * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z);
    double g_z = msg.orientation.w * msg.orientation.w - msg.orientation.x * msg.orientation.x - msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z;
    reading.linear.x = msg.linear_acceleration.x - g_x * GRAVITY;
    reading.linear.y = msg.linear_acceleration.y - g_y * GRAVITY;
    reading.linear.z = msg.linear_acceleration.z - g_z * GRAVITY;
  }
  reading.angular.x = msg.angular_velocity.x;
  reading.angular.y = msg.angular_velocity.y;
  reading.angular.z = msg.angular_velocity.z;

  sendMovement(reading);
}

std::string VelocityConverterIMU::getName()
{
  return topic_;
}