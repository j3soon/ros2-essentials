#!/bin/bash
set -e

if [ -z "$ISAAC_ROS" ] || [ "$ISAAC_ROS" != "YES" ]; then
    echo "Skipping Isaac ROS installation (ISAAC_ROS is not set or not 'YES')"
    exit 0
fi

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

if [ "$TARGETARCH" == "arm64" ]; then
    echo "Not Implemented: Isaac ROS installation for architecture: $TARGETARCH"
    exit 1
fi

if [ "$ISAAC_ROS" = "YES" ]; then
    echo "Installing Isaac ROS..."

    echo "Add Isaac apt repository..."
    # Ref: https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
    sudo apt-get update && sudo rm -rf /var/lib/apt/lists/* || exit 1

    echo "Setup Jetson debian repositories..."
    # Ref: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/e516c452bda914b631dac626e457594b6b73c2ea/docker/Dockerfile.base#L215-L226
    sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
    if [[ ${TARGETARCH} == 'arm64' ]]; then
        echo 'deb https://repo.download.nvidia.com/jetson/common r36.4 main' | sudo tee -a /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
        echo 'deb https://repo.download.nvidia.com/jetson/t234 r36.4 main' | sudo tee -a /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
    elif [[ ${TARGETARCH} == 'amd64' ]]; then
        sudo add-apt-repository "deb http://repo.download.nvidia.com/jetson/x86_64/$(lsb_release -cs) r36.4 main"
    else
        echo "Unrecognized platform: ${TARGETARCH}"
        exit 1
    fi
    sudo apt-get update && sudo rm -rf /var/lib/apt/lists/* || exit 1

    echo "Install VPI..."
    # Ref: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/e516c452bda914b631dac626e457594b6b73c2ea/docker/Dockerfile.base#L240C1-L257C13
    sudo apt-get update && sudo apt-get install -y \
        libnvvpi3 \
        vpi3-dev \
        && sudo rm -rf /var/lib/apt/lists/* || exit 1
    # TODO: Add VPI installation for arm64
    # Ref: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_vpi.html

    echo "Cloning Isaac ROS common..."
    # Ref: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html
    mkdir -p ${ISAAC_ROS_WS}/src
    cd ${ISAAC_ROS_WS}/src
    git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
else
    echo "Error: Unsupported Isaac ROS: $ISAAC_ROS"
    exit 1
fi

echo "Creating Isaac ROS directories with correct ownership to avoid permission issues after volume mount..."
sudo mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets \
    && sudo chown $USERNAME:$USERNAME ${ISAAC_ROS_WS}/isaac_ros_assets \
    || exit 1

echo "Isaac ROS installation completed successfully!"
