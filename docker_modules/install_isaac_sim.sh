#!/bin/bash
set -e

# Install Isaac Sim related components

# Check required environment variables
if [ -z "$USERNAME" ]; then
    echo "Error: USERNAME environment variable is required but not set"
    exit 1
fi
if [ -z "$TARGETARCH" ]; then
    echo "Error: TARGETARCH environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAAC_SIM_VERSION" ]; then
    echo "Skipping Isaac Sim installation as ISAAC_SIM_VERSION is not set"
    exit 0
fi
if [ -z "$ISAACSIM_PATH" ]; then
    echo "Error: ISAACSIM_PATH environment variable is required but not set"
    exit 1
fi

echo "Installing Isaac Sim components for architecture: $TARGETARCH"
echo "Isaac Sim version: $ISAAC_SIM_VERSION"

# Only install Isaac Sim components on amd64 architecture
if [ "$TARGETARCH" != "amd64" ]; then
    echo "Skipping Isaac Sim installation for architecture: $TARGETARCH (only supported on amd64)"
    exit 0
fi

echo "Installing 'libglu1-mesa' for Iray and 'libxrandr2' to support Isaac Sim WebRTC streaming..."
sudo apt-get update && sudo apt-get install -y \
    libglu1-mesa libxrandr2 \
    && sudo rm -rf /var/lib/apt/lists/* \
    || exit 1

if [ "$ISAAC_SIM_VERSION" = "5.1.0" ]; then
    echo "Installing Isaac Sim 5.1.0 (packaged with Python 3.11)..."
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html
    cd /tmp \
        && wget -q https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip \
        && 7z x "isaac-sim-standalone-5.1.0-linux-x86_64.zip" -o"$ISAACSIM_PATH" \
        && rm "isaac-sim-standalone-5.1.0-linux-x86_64.zip" \
        && cd "$ISAACSIM_PATH" \
        && ./post_install.sh \
        || exit 1

    # Note: Optional dependencies and the Isaac Sim ROS workspace are not installed to minimize image size
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#install-ros-2
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#setting-up-workspaces
else
    echo "Error: Unsupported Isaac Sim version: $ISAAC_SIM_VERSION"
    exit 1
fi

echo "Creating Isaac Sim directories with correct ownership to avoid permission issues after volume mount..."
sudo mkdir -p /isaac-sim && sudo chown $USERNAME:$USERNAME /isaac-sim || exit 1

if [ "$ISAAC_SIM_VERSION" = "5.1.0" ]; then
    echo "Creating Isaac Sim 5.1.0 specific directories with correct ownership to avoid permission issues after volume mount..."
    mkdir -p /isaac-sim/kit/cache \
        && mkdir -p /home/$USERNAME/.cache/ov \
        && mkdir -p /home/$USERNAME/.local/lib/python3.11/site-packages/omni/cache \
        && mkdir -p /home/$USERNAME/.cache/pip \
        && mkdir -p /home/$USERNAME/.cache/nvidia/GLCache \
        && mkdir -p /home/$USERNAME/.nv/ComputeCache \
        && mkdir -p /home/$USERNAME/.nvidia-omniverse/logs \
        && mkdir -p /home/$USERNAME/.local/lib/python3.11/site-packages/omni/logs \
        && mkdir -p /home/$USERNAME/.local/share/ov/data \
        && mkdir -p /home/$USERNAME/.local/lib/python3.11/site-packages/omni/data \
        && mkdir -p /home/$USERNAME/Documents \
        || exit 1
fi

echo "Isaac Sim installation completed successfully!"
