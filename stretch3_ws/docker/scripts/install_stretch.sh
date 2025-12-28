#!/bin/bash
set -e

echo "Installing PortAudio system headers..."
sudo apt-get update
sudo apt-get install -y portaudio19-dev libasound2-dev python3-dev

# Verify the header actually exists before continuing
if [ ! -f /usr/include/portaudio.h ]; then
    echo "CRITICAL ERROR: /usr/include/portaudio.h not found after installation."
    exit 1
else
    echo "SUCCESS: portaudio.h found. Proceeding..."
fi

# We install this separately first to catch errors early
echo "Pre-installing pyaudio wheel..."
pip3 install --upgrade pip wheel setuptools
pip3 install pyaudio

# 1. Install Stretch Python API (Stretch Body)
# Ref: https://github.com/hello-robot/stretch_body
echo "Installing Stretch Body..."
pip3 install hello-robot-stretch-body

# 2. Install System Dependencies for Stretch
# Common packages required for Stretch navigation and control
sudo apt-get update && sudo apt-get install -y \
    python3-opencv \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard

# 3. Setup Stretch ROS 2 Workspace
# We use the 'workspaces' directory seen in your ls output
WS_DIR="$HOME/workspaces/stretch_ws"
echo "Setting up Stretch ROS 2 Workspace at $WS_DIR..."
mkdir -p "$WS_DIR/src"

# Clone stretch_ros2 (Humble branch)
git clone -b humble https://github.com/hello-robot/stretch_ros2.git "$WS_DIR/src/stretch_ros2"

# Clone stretch_install (As requested, for reference/scripts)
git clone https://github.com/hello-robot/stretch_install.git "$WS_DIR/src/stretch_install"

# 4. Install ROS Dependencies via rosdep
source /opt/ros/humble/setup.bash
sudo apt-get update
rosdep update
rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y