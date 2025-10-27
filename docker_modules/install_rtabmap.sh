#!/bin/bash
set -e

if [ -z "$RTABMAP" ]; then
    echo "Skipping RTAB-Map installation as RTABMAP is not set"
    exit 0
fi

# Install RTAB-Map, TurtleBot3 and rqt steering
# This script is intended to be run inside the Dockerfile during build.
if [ "$RTABMAP" = "YES" ]; then
    echo "Installing RTAB-Map and turtlebot3 packages for ROS distro: ${ROS_DISTRO:-humble}"
    
    sudo apt-get update && sudo apt-get install -y \
        ros-${ROS_DISTRO}-rtabmap-ros \
        ros-${ROS_DISTRO}-turtlebot3* \
        ros-${ROS_DISTRO}-rqt-robot-steering \
        && sudo rm -rf /var/lib/apt/lists/*
fi

echo "RTAB-Map installation completed successfully!"