#!/bin/bash

cd /home/ros2-essentials/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf
# Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
xacro turtlebot3_burger_isaacsim.urdf.xacro > /home/ros2-essentials/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger_isaacsim.urdf
