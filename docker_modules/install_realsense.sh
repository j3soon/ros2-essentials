#!/bin/bash
set -eo pipefail

if [ -z "$REALSENSE" ] || [ "$REALSENSE" != "YES" ]; then
    echo "Skipping RealSense installation (REALSENSE is not set or not 'YES')"
    exit 0
fi

echo "Installing RealSense packages for ROS distro: ${ROS_DISTRO:-humble}"

# Ref: https://github.com/realsenseai/librealsense/blob/78cb605b11f5ba80176e7b8d70292f76ba625565/scripts/Docker/Dockerfile
LIBRS_VERSION=2.56.4
test -n "$LIBRS_VERSION"

DEBIAN_FRONTEND=noninteractive

sudo apt-get update
sudo apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    ca-certificates
sudo rm -rf /var/lib/apt/lists/*

cd /usr/src
sudo curl "https://codeload.github.com/realsenseai/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION" -o librealsense.tar.gz
sudo tar -zxf librealsense.tar.gz
sudo rm librealsense.tar.gz
sudo ln -s "/usr/src/librealsense-$LIBRS_VERSION" /usr/src/librealsense

cd /usr/src/librealsense
# When building examples (including graphics example), cmake will encounter the following OpenGL error:
#   CMake Error at examples/*/CMakeLists.txt:* Target * links to target "OpenGL::GL" but the target was not found.  Perhaps a find_package() call is missing for an target, or an ALIAS target is missing?
#
# librealsense v2.56.4 examples unconditionally link OpenGL::GL on Linux, but
# Ubuntu 22.04 with GLVND may only expose OpenGL::OpenGL and OpenGL::GLX.
# Add the legacy compatibility target before configuring so examples can build.
sudo sed -i '/# Check the platform and conditionally link OpenGL and libdl (for linux)/i \
find_package(OpenGL REQUIRED)\
if(NOT TARGET OpenGL::GL)\
    if(TARGET OpenGL::OpenGL AND TARGET OpenGL::GLX)\
        add_library(OpenGL::GL INTERFACE IMPORTED)\
        set_property(TARGET OpenGL::GL PROPERTY INTERFACE_LINK_LIBRARIES "OpenGL::OpenGL;OpenGL::GLX")\
    elseif(TARGET OpenGL::OpenGL)\
        add_library(OpenGL::GL INTERFACE IMPORTED)\
        set_property(TARGET OpenGL::GL PROPERTY INTERFACE_LINK_LIBRARIES OpenGL::OpenGL)\
    endif()\
endif()\
' examples/CMakeLists.txt
sudo mkdir build
cd build
# `-DFORCE_RSUSB_BACKEND=TRUE` to avoid the need of dkms with rsusb
# Ref: https://github.com/realsenseai/librealsense/issues/9931#issuecomment-964289692
# Ref: https://github.com/realsenseai/librealsense/issues/5212#issuecomment-552184604
# Using dkms requires installation on host, which could cause issues when the container and host OS versions differ.
# Using rsusb backend seems to work well enough and is also adopted in Isaac ROS.
# Ref: https://github.com/realsenseai/librealsense/discussions/13081#discussioncomment-9868325
# Ref: https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/blob/c9666b71e301967d505ad118a45c0aa89f5d72bd/docker/Dockerfile.realsense#L20
sudo cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DFORCE_RSUSB_BACKEND=TRUE \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ..
sudo make -j"$(($(nproc)-1))" all
sudo make install

sudo cp -a /opt/librealsense/. /usr/local/
sudo cp /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo cp /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common

# Custom installs
sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-point-cloud-transport \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-launch-pytest
sudo rm -rf /var/lib/apt/lists/*

echo "RealSense installation completed successfully!"
