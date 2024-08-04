#!/bin/bash -e

source $ROS2_WS/install/setup.bash

{
    ros2 launch vlp_cartographer vlp_driver.launch.py
} <&0 &

{
    ros2 launch vlp_cartographer cartographer_demo.launch.py using_odom:=False
} <&0

wait