#!/bin/bash

cd /home/ros2-essentials/aloha_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf
# Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
xacro vx300s.urdf.xacro > /home/ros2-essentials/aloha_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf
