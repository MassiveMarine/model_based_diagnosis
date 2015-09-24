//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterIMU.h>
#include <tug_yaml/ProcessYaml.h>

VelocityConverterIMU::VelocityConverterIMU(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back, tug_observers::ObserverPluginBase* plugin_base) : VelocityConverter(call_back)
{
  std::string topic_name = ProcessYaml::getValue<std::string>("topic", params);
  imu_sub_ = plugin_base->subscribe(topic_name, 1, &VelocityConverterIMU::IMUCB, this);
}

void VelocityConverterIMU::IMUCB(const sensor_msgs::Imu& msg)
{
  MovementReading reading;
  reading.reading_time = msg.header.stamp;
  reading.linear.x = msg.linear_acceleration.x;
  reading.linear.y = msg.linear_acceleration.y;
  reading.linear.z = msg.linear_acceleration.z;
  reading.angular.x = msg.angular_velocity.x;
  reading.angular.y = msg.angular_velocity.y;
  reading.angular.z = msg.angular_velocity.z;

  sendMovement(reading);
}