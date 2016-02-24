#!/bin/bash
echo "will source '/opt/ros/indigo/setup.bash' and set ROS_MASTER/ROS_IP"
source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

REPO_PATH=`pwd` 
TEMP_PATH=`mktemp -d`
echo "will work in '"$TEMP_PATH"'"

mkdir $TEMP_PATH/src
cd $TEMP_PATH/src
catkin_init_workspace
cd ..
catkin_make

source ./devel/setup.bash

ln -s $REPO_PATH ./src/

echo "ROS_PACKAGE_PATH:"
echo $ROS_PACKAGE_PATH

cd $TEMP_PATH/src
git clone http://git.ist.tugraz.at/ais/utils.git
cd ..
