#!/bin/bash
set -e

if [ -z "$REALSENSE" ] || [ "$REALSENSE" != "YES" ]; then
    echo "Skipping RealSense installation (REALSENSE is not set or not 'YES')"
    exit 0
fi

# Install RealSense, TurtleBot3 packages and rqt steering
# This script is intended to be run inside the Dockerfile during build.
if [ "$REALSENSE" = "YES" ]; then
    echo "Installing RealSense packages for ROS distro: ${ROS_DISTRO:-humble}"

    # Add RealSense apt repository and key.
    # Ref: https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
        gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list

    sudo apt-get update && sudo apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg \
        && sudo rm -rf /var/lib/apt/lists/*

    # Installing ros packages seem to automatically installs the required packages above.
    # Ref: https://github.com/realsenseai/realsense-ros#installation-on-ubuntu
    sudo apt-get update && sudo apt-get install -y \
        ros-${ROS_DISTRO}-point-cloud-transport \
        ros-${ROS_DISTRO}-librealsense2* \
        ros-${ROS_DISTRO}-realsense2-* \
        ros-${ROS_DISTRO}-turtlebot3* \
        ros-${ROS_DISTRO}-rqt-robot-steering \
        ros-${ROS_DISTRO}-launch-pytest \
        && sudo rm -rf /var/lib/apt/lists/*

    echo "RealSense installation completed successfully!"
fi
