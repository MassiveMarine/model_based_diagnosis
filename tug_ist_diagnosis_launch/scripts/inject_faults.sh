#!/bin/sh
echo "-------Increasing CPU---------"
rostopic pub -1 /cpu_usg std_msgs/Int32 1
sleep 3
echo "-------Reseting CPU---------"
rostopic pub -1 /cpu_usg std_msgs/Int32 0
sleep 4
echo "-------Increasing FRQ---------"
rostopic pub -1 /frq_inc std_msgs/Int32 1
sleep 3
echo "-------Reseting FRQ---------"
rostopic pub -1 /frq_inc std_msgs/Int32 0
sleep 4
echo "-------Changing Memory--------"
rostopic pub -1 /mem_usg std_msgs/Int32 1
echo "-------waiting for memeroy reseting--------"
sleep 4
#echo "-------Killing Node Teleop_joy--------"
#rosnode kill /teleop_joy
#sleep 2
#echo "-------Running node Teleop_joy--------"
#rosrun teleop_joy teleop_joy
