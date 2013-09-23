#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"

ros::Publisher imu_pitch_pub,imu_roll_pub,imu_yaw_pub,imu_gyro_pub,servo2_pub,odom_roll_pub, odom_pitch_pub, odom_yaw_pub;
int is_noise;


void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    tf::Quaternion q;
    //btQuaternion btq;
    double roll, pitch, yaw;
   /*double x,y,z,w;
    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;

    roll = 0.0;
    pitch = 0.0;*/
    //yaw = tf::getYaw(msg->orientation);
    //btq = msg->orientation;
    tf::quaternionMsgToTF(msg->orientation, q);
    //tf::Quaternion q1;
    //q = q.normalize();
    //q = tf::Quaternion::normalize(q);
    //tf::Quaternion q1 = q.normalized();
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //ROS_INFO("q.x=%lf,q.y=%lf,q.z=%lf,q.w=%f, imu.time=%lf, imu.roll= %lf, imu.pitch= %lf, imu.yaw= %lf",x,y,z,w, msg->header.stamp.toSec(), roll, pitch, yaw);
    //ROS_INFO("imu.time=%lf, imu.roll= %lf, imu.pitch= %lf, imu.yaw= %lf",msg->header.stamp.toSec(), roll, pitch, yaw);
	std_msgs::Float64 u;

    u.data = roll;
    imu_roll_pub.publish(u);

    u.data = pitch;
    imu_pitch_pub.publish(u);

    u.data = yaw;
    imu_yaw_pub.publish(u);

    u.data = msg->angular_velocity.z;
    imu_gyro_pub.publish(u);

}
void servo2Callback(const std_msgs::Float64ConstPtr& servo2_msg)
{
   std_msgs::Float64 u;
	 if(is_noise==0)
     u.data=servo2_msg->data;
   else
     u.data = 0.111111;
   servo2_pub.publish(u);
		
}

void odomCallback(const nav_msgs::OdometryPtr& msg)
{
    tf::Quaternion q;
    //Quaterniond r;
    double roll, pitch, yaw, sec, nsec;
    /*double x,y,z,w;
    x = msg->pose.pose.orientation.x;
    y = msg->pose.pose.orientation.y;
    z = msg->pose.pose.orientation.z;
    w = msg->pose.pose.orientation.w;
    roll = 0.0;
    pitch = 0.0;*/
    //yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    //q = q.normalize();
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ros::Time t;
    t = msg->header.stamp;
    sec = t.toSec();
    //ROS_INFO("q.x=%lf,q.y=%lf,q.z=%lf,q.w=%f, odom.time=%lf, odom.roll= %lf, odom.pitch= %lf, odom.yaw= %lf",x,y,z,w,msg->header.stamp.toSec(), roll, pitch, yaw);
    //ROS_INFO("odom.time=%lf, odom.roll= %lf, odom.pitch= %lf, odom.yaw= %lf",msg->header.stamp.toSec(), roll, pitch, yaw);
		std_msgs::Float64 u;

    u.data = roll;
    odom_roll_pub.publish(u);

    u.data = pitch;
    odom_pitch_pub.publish(u);

    u.data = yaw;
    odom_yaw_pub.publish(u);
}


void noiseCallback(const std_msgs::Float64ConstPtr& noise_msg)
{  
   if(noise_msg->data==0.0)
   		is_noise = 0;
	 else
			is_noise = 1;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "rpy_4_odom_imu_map_node");

  ros::NodeHandle n;
  ros::Publisher map_yaw_pub;
  tf::TransformListener listener;

  is_noise = 0;

  ros::Subscriber imu_sub = n.subscribe("/imu_data", 100, imuCallback);
  ros::Subscriber servo2_sub = n.subscribe("/robotis/servo_la_servo2", 100, servo2Callback);
  ros::Subscriber noise_sub = n.subscribe("/noise", 100, noiseCallback);
  ros::Subscriber odom_sub = n.subscribe("/pose", 100, odomCallback);

  servo2_pub = n.advertise<std_msgs::Float64>("servo2", 100);
  imu_roll_pub = n.advertise<std_msgs::Float64>("roll_imu", 100);
  imu_pitch_pub = n.advertise<std_msgs::Float64>("pitch_imu", 100);
  imu_yaw_pub = n.advertise<std_msgs::Float64>("yaw_imu", 100);
  imu_gyro_pub = n.advertise<std_msgs::Float64>("gyro_imu", 100);

  odom_roll_pub = n.advertise<std_msgs::Float64>("roll_odom", 100);
  odom_pitch_pub = n.advertise<std_msgs::Float64>("pitch_odom", 100);
  odom_yaw_pub = n.advertise<std_msgs::Float64>("yaw_odom", 100);

  map_yaw_pub = n.advertise<std_msgs::Float64>("yaw_map", 100);
  
  ros::Rate rate(10.0);

  ROS_INFO("RPY node is now running");
  while (n.ok()){
    /*tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    double mapYaw = tf::getYaw(transform.getRotation());
    ROS_INFO("map_yaw = %lf ", mapYaw);

    std_msgs::Float64 u;
    u.data = mapYaw;
    map_yaw_pub.publish(u);
*/
   rate.sleep();
   ros::spinOnce();
  }
  return 0;
};
