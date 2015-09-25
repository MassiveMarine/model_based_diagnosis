//
// Created by clemens on 08.09.15.
//

#include <tug_velocity_observer/VelocityConverterTf.h>
#include <tug_yaml/ProcessYaml.h>

VelocityConverterTf::VelocityConverterTf(XmlRpc::XmlRpcValue params, boost::function<void (MovementReading)> call_back) : VelocityConverterPoseStamped(call_back), tf_update_rate_(1.0)
{
  target_frame_ = ProcessYaml::getValue<std::string>("target_frame", params);
  base_frame_ = ProcessYaml::getValue<std::string>("base_frame", params);
}

void VelocityConverterTf::run()
{
  while(ros::ok())
  {
    tf::StampedTransform transform;
    try{
      tf_listener_.lookupTransform(target_frame_, base_frame_,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR_STREAM("transformation exception " << ex.what());
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = transform.frame_id_;
    pose_stamped.header.stamp = transform.stamp_;
    pose_stamped.pose.position.x = transform.getOrigin().x();
    pose_stamped.pose.position.y = transform.getOrigin().y();
    pose_stamped.pose.position.z = transform.getOrigin().z();
    pose_stamped.pose.orientation.x = transform.getRotation().x();
    pose_stamped.pose.orientation.y = transform.getRotation().y();
    pose_stamped.pose.orientation.z = transform.getRotation().z();
    pose_stamped.pose.orientation.w = transform.getRotation().w();
    PoseStampedCB(pose_stamped);

    tf_update_rate_.sleep();
  }
}

std::string VelocityConverterTf::getName()
{
  return base_frame_+ "_" + target_frame_;
}