(define (problem prob)(:domain test_repair_domain)(:requirements :strips :typing :negative-preconditions)(:objects scan jaguar_node sensor_head laser_alignment pc hokuyo kinect router thermal_camera jaguar no_dev1 no_dev2 hokuyo_node odom )(:init (component scan)(ok scan)(component jaguar_node)(not_running jaguar_node)(component sensor_head)(on sensor_head)(component laser_alignment)(on laser_alignment)(component pc)(on pc)(component hokuyo)(on hokuyo)(component kinect)(on kinect)(component router)(on router)(component thermal_camera)(on thermal_camera)(component jaguar)(on jaguar)(component no_dev1)(on no_dev1)(component no_dev2)(on no_dev2)(component hokuyo_node)(running hokuyo_node)(component odom)(not_ok odom)(good jaguar)(good hokuyo)(good hokuyo_node)(bad jaguar_node))(:goal (and (good jaguar)(good hokuyo)(good hokuyo_node)(good jaguar_node))))