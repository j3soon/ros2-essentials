#!/bin/bash
set -e

if [ "$RTABMAP" != "YES" ] && [ "$RTABMAP" != "yes" ] && [ "$RTABMAP" != "y" ] && [ "$RTABMAP" != "Y" ]; then
    echo "Skipping RTAB-Map installation (set RTABMAP to YES/yes/y to enable)"
    exit 0
fi

# Install RTAB-Map, TurtleBot3 and rqt steering
# This script is intended to be run inside the Dockerfile during build.
echo "Installing RTAB-Map and turtlebot3 packages for ROS distro: ${ROS_DISTRO:-humble}"

sudo apt-get update && sudo apt-get install -y \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    && sudo rm -rf /var/lib/apt/lists/*

echo "RTAB-Map installation completed successfully!"
