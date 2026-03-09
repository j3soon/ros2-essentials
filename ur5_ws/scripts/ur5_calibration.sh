#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
UR5_WORKSPACE_DIR="${SCRIPT_DIR}/.."

# Reference:
# - https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_calibration/doc/usage.html
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=192.168.56.101 \
    target_filename:="${UR5_WORKSPACE_DIR}/src/Universal_Robots_ROS2_Description/config/ur5/default_kinematics.yaml"