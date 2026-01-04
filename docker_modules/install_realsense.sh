#!/bin/bash
set -e

if [ -z "$REALSENSE" ]; then
    echo "Skipping RealSense installation as REALSENSE is not set"
    exit 0
fi

# Install RealSense, TurtleBot3 packages and rqt steering
# This script is intended to be run inside the Dockerfile during build.
if [ "$REALSENSE" = "YES" ]; then
    echo "Installing RealSense ros packages for ROS distro: ${ROS_DISTRO:-humble}"

    sudo apt-get update && sudo apt-get install -y \
        ros-humble-point-cloud-transport \
        ros-humble-librealsense2* \
        ros-humble-realsense2-* \
        ros-${ROS_DISTRO}-turtlebot3* \
        ros-${ROS_DISTRO}-rqt-robot-steering \
        ros-${ROS_DISTRO}-launch-pytest \
        && sudo rm -rf /var/lib/apt/lists/*
fi

echo "RealSense installation completed successfully!"
