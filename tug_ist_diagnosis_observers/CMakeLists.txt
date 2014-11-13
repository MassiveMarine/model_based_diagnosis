cmake_minimum_required(VERSION 2.8.3)
project(tug_ist_diagnosis_observers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roslib rospy rosgraph std_msgs sensor_msgs tug_ist_diagnosis_msgs rostopic diagnostic_msgs)

catkin_package()
#catkin_python_setup()
