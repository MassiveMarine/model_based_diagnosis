#!/bin/sh
echo "-------waiting for 10 secs---------"
sleep 10
echo "-------Increasing CPU---------"
rostopic pub -1 /cpu_inc std_msgs/Int32 1
sleep 5
echo "-------Reseting CPU---------"
rostopic pub -1 /cpu_inc std_msgs/Int32 0
sleep 20
echo "-------Changing Memory--------"
rostopic pub -1 /mem_inc std_msgs/Int32 1
sleep 5
echo "-------reseting Mem--------"
rostopic pub -1 /mem_inc std_msgs/Int32 0
#sleep 4
#echo "-------Increasing FRQ---------"
#rostopic pub -1 /frq_inc std_msgs/Int32 1
#sleep 3
#echo "-------Reseting FRQ---------"
#rostopic pub -1 /frq_inc std_msgs/Int32 0
#sleep 4
#echo "-------Changing Memory--------"
#rostopic pub -1 /mem_inc std_msgs/Int32 1
#sleep 4
#rostopic pub -1 /mem_inc std_msgs/Int32 0
#echo "-------reseting Mem--------"
#sleep 4
#echo "-------Changing Memory--------"
#rostopic pub -1 /mem_inc std_msgs/Int32 1
#sleep 5
#rostopic pub -1 /mem_inc std_msgs/Int32 0
#echo "-------reseting Mem--------"
#sleep 4
#echo "-------Changing Memory--------"
#rostopic pub -1 /mem_inc std_msgs/Int32 1
#sleep 4
#rostopic pub -1 /mem_inc std_msgs/Int32 0
#echo "-------reseting Mem--------"
#echo "-------waiting for 10 secs---------"
#sleep 10
#echo "-------Increasing FRQ---------"
#rostopic pub -1 /frq_inc std_msgs/Int32 1
#sleep 3
#echo "-------Reseting FRQ---------"
#rostopic pub -1 /frq_inc std_msgs/Int32 0
#sleep 5
#echo "-------Killing Node Teleop_joy--------"
#rosnode kill /base_hokuyo_node
#sleep 3
#echo "-------Running node Teleop_joy--------"
#roslaunch tug_ist_diagnosis_launch base_hokuyo_node.launch

