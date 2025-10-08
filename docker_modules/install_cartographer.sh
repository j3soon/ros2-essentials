#!/bin/bash
set -e

if [ -z "$CARTOGRAPHER" ]; then
    echo "Skipping Cartographer installation as CARTOGRAPHER is not set"
    exit 0
fi

# Install Cartographer, TurtleBot3 packages and rqt steering
# This script is intended to be run inside the Dockerfile during build.
if [ "$CARTOGRAPHER" = "YES" ]; then
    echo "Installing cartographer and turtlebot3 packages for ROS distro: ${ROS_DISTRO:-humble}"

    sudo apt-get update && sudo apt-get install -y \
        ros-${ROS_DISTRO}-cartographer \
        ros-${ROS_DISTRO}-turtlebot3* \
        ros-${ROS_DISTRO}-rqt-robot-steering \
        && sudo rm -rf /var/lib/apt/lists/*
fi

echo "Cartographer installation completed successfully!"
