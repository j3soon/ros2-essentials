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
    && sudo rm -rf /var/lib/apt/lists/*

if [ "$ISAAC_SIM_VERSION" = "4.5.0" ]; then
    echo "Installing Isaac Sim Compatibility Checker 4.5.0..."
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html#isaac-sim-compatibility-checker
    python3 -V | grep "Python 3.10" \
        && cd /tmp \
        && wget -q https://download.isaacsim.omniverse.nvidia.com/isaac-sim-comp-check%404.5.0-rc.6%2Brelease.675.f1cca148.gl.linux-x86_64.release.zip \
        && unzip "isaac-sim-comp-check@4.5.0-rc.6+release.675.f1cca148.gl.linux-x86_64.release.zip" -d ~/isaac-sim-comp-check \
        && rm "isaac-sim-comp-check@4.5.0-rc.6+release.675.f1cca148.gl.linux-x86_64.release.zip"
    echo "Installing Isaac Sim 4.5.0 (requires Python 3.10)..."
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html
    python3 -V | grep "Python 3.10" \
        && cd /tmp \
        && wget -q https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.linux-x86_64.release.zip \
        && unzip "isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.linux-x86_64.release.zip" -d "$ISAACSIM_PATH" \
        && rm "isaac-sim-standalone@4.5.0-rc.36+release.19112.f59b3005.gl.linux-x86_64.release.zip" \
        && cd "$ISAACSIM_PATH" \
        && ./post_install.sh
else
    echo "Error: Unsupported Isaac Sim version: $ISAAC_SIM_VERSION"
    exit 1
fi

echo "Fixing SciPy (in base image) incompatibility with NumPy version (in Isaac Sim) numpy==1.26.0..."
pip install scipy==1.14.1 numpy==1.26.0

echo "Isaac Sim installation completed successfully!"
