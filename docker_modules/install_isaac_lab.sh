#!/bin/bash
set -e

if [ -z "$ISAAC_LAB_VERSION" ]; then
    echo "Skipping Isaac Lab installation as ISAAC_LAB_VERSION is not set"
    exit 0
fi

# Install Isaac Lab

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
    echo "Error: ISAAC_SIM_VERSION environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAACSIM_PATH" ]; then
    echo "Error: ISAACSIM_PATH environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAACLAB_PATH" ]; then
    echo "Error: ISAACLAB_PATH environment variable is required but not set"
    exit 1
fi

echo "Installing Isaac Lab for architecture: $TARGETARCH"
echo "Isaac Lab version: $ISAAC_LAB_VERSION"

# Only install Isaac Lab on amd64 architecture
if [ "$TARGETARCH" != "amd64" ]; then
    echo "Skipping Isaac Lab installation for architecture: $TARGETARCH (only supported on amd64)"
    exit 0
fi

ISAAC_LAB_GIT_REF=""
if [ "$ISAAC_LAB_VERSION" = "2.3.0" ]; then
    ISAAC_LAB_GIT_REF="v2.3.0"
elif [ "$ISAAC_LAB_VERSION" = "2.3.2" ]; then
    ISAAC_LAB_GIT_REF="v2.3.2"
elif [ "$ISAAC_LAB_VERSION" = "develop" ]; then
    ISAAC_LAB_GIT_REF="develop"
fi

if [ -n "$ISAAC_LAB_GIT_REF" ]; then
    echo "Installing Isaac Lab from git ref: $ISAAC_LAB_GIT_REF..."
    # Ref: https://isaac-sim.github.io/IsaacLab/source/setup/installation/binaries_installation.html
    sudo apt-get update && sudo apt-get install -y \
        cmake build-essential \
        && sudo rm -rf /var/lib/apt/lists/* \
        || exit 1
    # Note that the flatdict patch is for preventing
    #    ModuleNotFoundError: No module named 'pkg_resources'
    # Ref: https://github.com/isaac-sim/IsaacLab/issues/4576#issuecomment-4083197347
    git clone -b "$ISAAC_LAB_GIT_REF" https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB_PATH" \
        && sed -i 's/"flatdict==4.0.1"/"flatdict==4.1.0"/' "$ISAACLAB_PATH/source/isaaclab/setup.py" \
        && cd "$ISAACLAB_PATH" \
        && ln -s "$ISAACSIM_PATH" _isaac_sim \
        && ./isaaclab.sh --install \
        || exit 1
else
    echo "Error: Unsupported Isaac Lab version: $ISAAC_LAB_VERSION"
    exit 1
fi
echo "Isaac Lab installation completed successfully!"
