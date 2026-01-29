#!/bin/bash
set -e

if [ -z "$CUDA_TOOLKIT_VERSION" ]; then
    echo "Skipping CUDA Toolkit installation as CUDA_TOOLKIT_VERSION is not set"
    exit 0
fi

# Install CUDA Toolkit related components

# Check required environment variables
if [ -z "$TARGETARCH" ]; then
    echo "Error: TARGETARCH environment variable is required but not set"
    exit 1
fi

echo "Installing CUDA Toolkit components for architecture: $TARGETARCH"
echo "CUDA Toolkit version: $CUDA_TOOLKIT_VERSION"

# Only install CUDA Toolkit components on amd64 architecture
# We chose to use `deb (network)` installation for simplicity of changing across minor versions (no need to set driver version)
# We may need to use other installation methods if we want to change the PATCH version (e.g. `deb (local)`)
if [ "$TARGETARCH" == "amd64" ]; then
    # Ref: https://developer.nvidia.com/cuda-toolkit-archive
    # Ref: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#network-repo-installation-for-ubuntu
    # Ref: https://developer.nvidia.com/cuda-12-6-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network
    cd /tmp
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
    dpkg -i cuda-keyring_1.1-1_all.deb
    rm cuda-keyring_1.1-1_all.deb
    apt-get update
    apt-get -y install cuda-toolkit-${CUDA_TOOLKIT_VERSION//./-}
    rm -rf /var/lib/apt/lists/*

    # Note: CUDA samples are not downloaded to minimize image size
    # Ref: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#recommended-actions
    # Ref: https://github., and can be installed easily later
elif [ "$TARGETARCH" == "arm64" ]; then
    echo "Not Implemented: CUDA Toolkit installation for architecture: $TARGETARCH"
    exit 1
else
    echo "Skipping CUDA Toolkit installation for architecture: $TARGETARCH"
    exit 0
fi

echo "CUDA Toolkit installation completed successfully!"
