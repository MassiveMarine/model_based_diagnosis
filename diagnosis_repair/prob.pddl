(define (problem prob)(:domain test_repair_domain)(:requirements :strips :typing :negative-preconditions)(:objects sensor_head laser_alignment pc laser kinect router thermal_camera jaguar_base scan odometry sensor_head_topic image kinect_node jaguar_node sensor_head_node laser_node )(:init (component sensor_head)(on sensor_head)(component laser_alignment)(on laser_alignment)(component pc)(on pc)(component laser)(on laser)(component kinect)(on kinect)(component router)(on router)(component thermal_camera)(on thermal_camera)(component jaguar_base)(on jaguar_base)(component scan)(ok scan)(component odometry)(ok odometry)(component sensor_head_topic)(ok sensor_head_topic)(component image)(not_ok image)(component kinect_node)(not_running kinect_node)(component jaguar_node)(running jaguar_node)(component sensor_head_node)(running sensor_head_node)(component laser_node)(running laser_node)(good jaguar_base)(good laser)(good kinect)(good sensor_head)(good jaguar_node)(good laser_node)(good sensor_head_node)(bad kinect_node))(:goal (and (good jaguar_base)(good laser)(good kinect)(good sensor_head)(good jaguar_node)(good laser_node)(good sensor_head_node)(good kinect_node))))