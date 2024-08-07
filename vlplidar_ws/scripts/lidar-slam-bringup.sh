#!/bin/bash -e

source $ROS2_WS/install/setup.bash
ros2 launch vlp_cartographer cartographer_demo.launch.py using_odom:=False