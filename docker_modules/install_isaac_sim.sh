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

echo "Installing Isaac Sim components for architecture: $TARGETARCH"
echo "Isaac Sim version: $ISAAC_SIM_VERSION"

# Only install Isaac Sim components on amd64 architecture
if [ "$TARGETARCH" = "amd64" ]; then
    echo "Installing libxrandr2 to support Isaac Sim WebRTC streaming..."
    sudo apt-get update && sudo apt-get install -y \
        libxrandr2 \
        && sudo rm -rf /var/lib/apt/lists/*

    echo "Installing Isaac Sim (requires Python 3.10)..."
    # Note that installing Isaac Sim with pip is experimental, keep this in mind when unexpected error occurs
    # TODO: Remove the note above when it is no longer experimental
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#installation-using-pip
    python3 -V | grep "Python 3.10" \
        && pip install isaacsim[all]==$ISAAC_SIM_VERSION --extra-index-url https://pypi.nvidia.com \
        && pip install isaacsim[extscache]==$ISAAC_SIM_VERSION --extra-index-url https://pypi.nvidia.com

    echo "Fixing SciPy (in base image) incompatibility with NumPy version (in Isaac Sim) numpy==1.26.0..."
    pip install scipy==1.14.1 numpy==1.26.0

    echo "Creating isaac sim cache directory with correct ownership..."
    sudo mkdir -p /isaac-sim/kit/cache \
        && sudo chown -R $USERNAME:$USERNAME /isaac-sim/kit/cache

    echo "Creating Isaac Sim user cache directories..."
    mkdir -p /home/$USERNAME/.cache/ov \
        && mkdir -p /home/$USERNAME/.cache/pip \
        && mkdir -p /home/$USERNAME/.cache/nvidia/GLCache \
        && mkdir -p /home/$USERNAME/.nv/ComputeCache \
        && mkdir -p /home/$USERNAME/.nvidia-omniverse/logs \
        && mkdir -p /home/$USERNAME/.local/share/ov/data \
        && mkdir -p /home/$USERNAME/Documents

    echo "Isaac Sim installation completed successfully!"
else
    echo "Skipping Isaac Sim installation for architecture: $TARGETARCH (only supported on amd64)"
fi
