#!/bin/bash
set -e

if [ "$NV_OPENUSD" != "YES" ] && [ "$NV_OPENUSD" != "yes" ] && [ "$NV_OPENUSD" != "y" ] && [ "$NV_OPENUSD" != "Y" ]; then
    echo "Skipping NVIDIA OpenUSD tools installation (set NV_OPENUSD to YES/yes/y to enable)"
    exit 0
fi

# Install NVIDIA OpenUSD tools
# This script is intended to be run inside the Dockerfile during build.
if [ -z "$TARGETARCH" ]; then
    echo "Error: TARGETARCH environment variable is required but not set"
    exit 1
fi
if [ -z "$USERNAME" ]; then
    echo "Error: USERNAME environment variable is required but not set"
    exit 1
fi

# The current NVIDIA OpenUSD Linux binary package is x86_64-only.
if [ "$TARGETARCH" != "amd64" ]; then
    echo "Skipping NVIDIA OpenUSD tools installation for architecture: $TARGETARCH (only supported on amd64)"
    exit 0
fi

# Ref: https://developer.nvidia.com/openusd#section-getting-started
NV_OPENUSD_URL="https://developer.nvidia.com/downloads/usd/usd_binaries/25.08/usd.py312.manylinux_2_35_x86_64.usdview.release-v25.08.71e038c1.zip"
NV_OPENUSD_PATH="$HOME/nvidia/openusd"

echo "Downloading NVIDIA OpenUSD tools from: $NV_OPENUSD_URL"
echo "Install path: $NV_OPENUSD_PATH"

# Install required dependencies and missing dependencies through trial-and-error.
# Ref: https://docs.omniverse.nvidia.com/usd/latest/usdview/quickstart.html
sudo apt-get update && sudo apt-get install -y \
    wget \
    unzip \
    libxkbcommon-x11-0 \
    libxcb-xinerama0 \
    libxcb-image0 \
    libxcb-shape0 \
    libxcb-render-util0 \
    libxcb-icccm4 \
    libxcb-keysyms1 \
    libgomp1 \
    libglib2.0-0 \
    libgl1 \
    libegl1 \
    libfontconfig1 \
    libdbus-1-3 \
    libxcb-cursor0 \
    && sudo rm -rf /var/lib/apt/lists/*

rm -rf "$NV_OPENUSD_PATH"
mkdir -p "$NV_OPENUSD_PATH"

TMP_DIR=$(mktemp -d)
wget -q -O "$TMP_DIR/openusd.zip" "$NV_OPENUSD_URL"
unzip -q "$TMP_DIR/openusd.zip" -d "$NV_OPENUSD_PATH"

# Fix executable permissions in the distributed package.
chmod +x "$NV_OPENUSD_PATH"/scripts/usd*
chmod +x "$NV_OPENUSD_PATH"/scripts/sdf*
chmod +x "$NV_OPENUSD_PATH"/scripts/check_python_dependencies.sh
chmod +x "$NV_OPENUSD_PATH"/python/bin/python3
chmod +x "$NV_OPENUSD_PATH"/python/bin/python3.12
chmod +x "$NV_OPENUSD_PATH"/bin/usd*
chmod +x "$NV_OPENUSD_PATH"/bin/sdf*

# Persist PATH updates for interactive shells via .bashrc.
NV_OPENUSD_SCRIPTS_PATH="$NV_OPENUSD_PATH/scripts"
cat >> "/home/$USERNAME/.bashrc" <<EOF

# NV_OPENUSD PATH
if [ -d "$NV_OPENUSD_SCRIPTS_PATH" ]; then
    export PATH="$NV_OPENUSD_SCRIPTS_PATH:\$PATH"
fi
EOF

rm -rf "$TMP_DIR"

echo "NVIDIA OpenUSD tools installed successfully."

echo "NVIDIA OpenUSD tools installation completed!"
