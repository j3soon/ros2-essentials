#!/bin/bash -e

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 \
    use_rg2_gripper:=true \
    robot_ip:=192.168.56.101 \
    launch_rviz:=true
