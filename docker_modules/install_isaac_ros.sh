#!/bin/bash
set -e

# Install Isaac ROS related components

# Check required environment variables
if [ -z "$USERNAME" ]; then
    echo "Error: USERNAME environment variable is required but not set"
    exit 1
fi
if [ -z "$TARGETARCH" ]; then
    echo "Error: TARGETARCH environment variable is required but not set"
    exit 1
fi
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS_DISTRO environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAAC_ROS" ]; then
    echo "Skipping Isaac ROS installation as ISAAC_ROS is not set"
    exit 0
fi
if [ -z "$ISAAC_ROS_WS" ]; then
    echo "Error: ISAAC_ROS_WS environment variable is required but not set"
    exit 1
fi
if [ "$CUDA_TOOLKIT_VERSION" != "12.6" ]; then
    # Ref: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docker/Dockerfile.base
    echo "Error: CUDA_TOOLKIT_VERSION must be 12.6 for Isaac ROS"
    # Other versions may be supported, but we haven't tested them yet.
    exit 1
fi

echo "Installing Isaac ROS components for architecture: $TARGETARCH"
echo "Isaac ROS: $ISAAC_ROS"

# Only install Isaac ROS components on supported architectures
if [ "$TARGETARCH" != "amd64" ] && [ "$TARGETARCH" != "arm64" ]; then
    echo "Skipping Isaac ROS installation for architecture: $TARGETARCH (only supported on amd64 and arm64)"
    exit 0
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo "Error: Unsupported ROS distribution $ROS_DISTRO"
    exit 1
fi

if [ "$ISAAC_ROS" = "YES" ]; then
    echo "Installing Isaac ROS..."
    # Ref: https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
    sudo apt-get update && sudo rm -rf /var/lib/apt/lists/* || exit 1

    # Ref: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html
    mkdir -p ${ISAAC_ROS_WS}/src
    cd ${ISAAC_ROS_WS}/src
    git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
else
    echo "Error: Unsupported Isaac ROS: $ISAAC_ROS"
    exit 1
fi

echo "Isaac ROS installation completed successfully!"
