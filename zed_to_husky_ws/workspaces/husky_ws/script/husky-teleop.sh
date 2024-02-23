#!/bin/bash -e

# This script is used to teleoperate the Husky robot.
# Reference: https://index.ros.org/r/teleop_twist_keyboard/

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/a200_0000/cmd_vel