(define (problem repair_goal)(:domain repair_domain)(:requirements :strips :typing :negative-preconditions)(:objects sensor_head laser_alignment pc hokuyo kinect router thermal_camera jaguar hokuyo_node1 jaguar_node1 kinect_node1 image scan odometry )(:init (component sensor_head)(on sensor_head)(component laser_alignment)(on laser_alignment)(component pc)(on pc)(component hokuyo)(on hokuyo)(component kinect)(on kinect)(component router)(on router)(component thermal_camera)(on thermal_camera)(component jaguar)(on jaguar)(component hokuyo_node1)(not_running hokuyo_node1)(component jaguar_node1)(running jaguar_node1)(component kinect_node1)(running kinect_node1)(component image)(ok image)(component scan)(not_ok scan)(component odometry)(ok odometry)(good jaguar)(good hokuyo)(good kinect)(good jaguar_node1)(good kinect_node1)(bad hokuyo_node1))(:goal (and (good jaguar)(good hokuyo)(good kinect)(good jaguar_node1)(good kinect_node1)(good hokuyo_node1))))