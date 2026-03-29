#!/bin/bash
set -e

if [ -z "$ISAAC_SIM_VERSION" ]; then
    echo "Skipping Isaac Sim installation as ISAAC_SIM_VERSION is not set"
    exit 0
fi

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
if [ -z "$ISAACSIM_PATH" ]; then
    echo "Error: ISAACSIM_PATH environment variable is required but not set"
    exit 1
fi

echo "Installing Isaac Sim components for architecture: $TARGETARCH"
echo "Isaac Sim version: $ISAAC_SIM_VERSION"

ISAAC_SIM_STANDALONE_ARCHIVE=""
ISAAC_SIM_SOURCE_PATH="/home/$USERNAME/IsaacSim"

# Only install Isaac Sim components on amd64 architecture
if [ "$TARGETARCH" != "amd64" ]; then
    echo "Skipping Isaac Sim installation for architecture: $TARGETARCH (only supported on amd64)"
    exit 0
fi

if [ "$ISAAC_SIM_VERSION" = "develop" ]; then
    echo "Isaac Sim source build enabled from branch: develop"

    echo "Installing build dependencies for Isaac Sim source build..."
    sudo apt-get update && sudo apt-get install -y \
        build-essential gcc-11 g++-11 \
        && sudo rm -rf /var/lib/apt/lists/* \
        || exit 1
fi

echo "Installing 'libglu1-mesa' for Iray and 'libxrandr2' to support Isaac Sim WebRTC streaming..."
sudo apt-get update && sudo apt-get install -y \
    libglu1-mesa libxrandr2 \
    && sudo rm -rf /var/lib/apt/lists/* \
    || exit 1

if [ "$ISAAC_SIM_VERSION" = "develop" ]; then
    echo "Cloning Isaac Sim source repository..."
    git clone -b develop https://github.com/isaac-sim/IsaacSim.git "$ISAAC_SIM_SOURCE_PATH" \
        && cd "$ISAAC_SIM_SOURCE_PATH" \
        && git lfs install \
        && git lfs pull \
        && touch .eula_accepted \
        && ./build.sh --config release \
        && ./repo.sh package --config release -m isaac-sim-standalone \
        || exit 1

    ISAAC_SIM_STANDALONE_ARCHIVE="$(
        find "$ISAAC_SIM_SOURCE_PATH/_build/packages" -maxdepth 1 -type f -name 'isaac-sim-standalone*release*.7z' \
            | sort \
            | tail -n 1
    )"
    if [ -z "$ISAAC_SIM_STANDALONE_ARCHIVE" ]; then
        echo "Error: Isaac Sim standalone package archive was not generated"
        exit 1
    fi
elif [ "$ISAAC_SIM_VERSION" = "5.1.0" ]; then
    echo "Installing Isaac Sim 5.1.0 (packaged with Python 3.11)..."
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html
    cd /tmp \
        && wget -q https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip \
        || exit 1

    ISAAC_SIM_STANDALONE_ARCHIVE="/tmp/isaac-sim-standalone-5.1.0-linux-x86_64.zip"
else
    echo "Error: Unsupported Isaac Sim version: $ISAAC_SIM_VERSION"
    exit 1
fi

if [ "$ISAAC_SIM_VERSION" = "develop" ]; then
    echo "Extracting standalone package to $ISAACSIM_PATH..."
    rm -rf "$ISAACSIM_PATH" \
        && mkdir -p "$ISAACSIM_PATH" \
        && 7z x "$ISAAC_SIM_STANDALONE_ARCHIVE" -o"$ISAACSIM_PATH" \
        || exit 1

    echo "Removing Isaac Sim source build artifacts after packaging..."
    rm -rf "$ISAAC_SIM_SOURCE_PATH" \
        && rm -rf "/home/$USERNAME/.cache/packman" \
        || exit 1
elif [ "$ISAAC_SIM_VERSION" = "5.1.0" ]; then
    # It's a bit unfortunate that we are currently manually compressing the source build and then extracting
    # it again to install, but without this process, the build will not be standalone (depends on `.cache`).
    echo "Extracting standalone package to $ISAACSIM_PATH..."
    rm -rf "$ISAACSIM_PATH" \
        && mkdir -p "$ISAACSIM_PATH" \
        && 7z x "$ISAAC_SIM_STANDALONE_ARCHIVE" -o"$ISAACSIM_PATH" \
        || exit 1

    cd "$ISAACSIM_PATH" \
        && ./post_install.sh \
        && rm -f "$ISAAC_SIM_STANDALONE_ARCHIVE" \
        || exit 1

    # Note: Optional dependencies and the Isaac Sim ROS workspace are not installed to minimize image size
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#install-ros-2
    # Ref: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#setting-up-workspaces
fi

echo "Creating Isaac Sim directories with correct ownership to avoid permission issues after volume mount..."
sudo mkdir -p /isaac-sim && sudo chown $USERNAME:$USERNAME /isaac-sim || exit 1

if [ "$ISAAC_SIM_VERSION" = "develop" ] || [ "$ISAAC_SIM_VERSION" = "5.1.0" ]; then
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
