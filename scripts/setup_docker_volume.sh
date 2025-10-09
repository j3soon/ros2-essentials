#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Create docker volume
docker volume create ros2-gazebo-cache >/dev/null
docker volume create ros2-isaac-sim-cache >/dev/null
docker volume create ros2-isaac-ros-assets >/dev/null

echo "Set up docker volume done."
