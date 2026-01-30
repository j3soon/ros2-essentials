#!/bin/bash
set -e

if [ -z "$ROS_DISTRO" ]; then
    echo "Skipping ROS installation as ROS_DISTRO is not set"
    exit 0
fi

# Install ROS related components
# Ref: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

# Note: Do not install additional ROS 2 packages here,
#       as they may not be required for all applications. 
#       Please install any other necessary packages in the Dockerfile.  

# Check required environment variables
if [ -z "$TARGETARCH" ]; then
    echo "Error: TARGETARCH environment variable is required but not set"
    exit 1
fi

echo "Installing ROS components for architecture: $TARGETARCH"

# Update and upgrade the system
apt-get update
apt-get upgrade -y

# Check the Ubuntu version supported by the ROS distribution
# Ref: https://docs.ros.org/en/humble/Releases.html
apt-get install -y lsb-release
UBUNTU_VERSION=$(lsb_release -rs)
case "$ROS_DISTRO:$UBUNTU_VERSION" in
    "foxy:20.04")
        echo "Installing ROS Foxy on Ubuntu 20.04..."
        ;;
    "humble:22.04")
        echo "Installing ROS Humble on Ubuntu 22.04..."
        ;;
    "jazzy:24.04")
        echo "Installing ROS Jazzy on Ubuntu 24.04..."
        ;;
    *)
        echo "Error: Unsupported ROS distribution $ROS_DISTRO with Ubuntu version $UBUNTU_VERSION"
        echo "Supported combinations are:"
        echo "  - ROS Foxy on Ubuntu 20.04"
        echo "  - ROS Humble on Ubuntu 22.04"
        echo "  - ROS Jazzy on Ubuntu 24.04"
        exit 1
        ;;
esac

# Set timezone
# Note: This step is not written in the ROS documentation,
#       but it is required to avoid tzdata prompts during installation.
DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# Set locale
apt-get install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
apt-get install -y software-properties-common
add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
apt-get install -y /tmp/ros2-apt-source.deb
rm -rf /tmp/ros2-apt-source.deb

# Install ROS 2 packages
apt-get update && apt-get upgrade -y || exit 1
apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-dev-tools

# Initialize rosdep
rosdep init

# Install CycloneDDS
# Ref:
# - https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cyclone-dds-tuning
# - https://github.com/ros2/rmw_cyclonedds/blob/406277084be4352fa48357047ed56be69e4942e5/README.md
# - https://github.com/eclipse-cyclonedds/cyclonedds/blob/ad48dc88a1da2e362c56ff1325d085ba514c0c74/README.md#run-time-configuration
apt-get install -y \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Install Gazebo
# Ref: 
# - https://gazebosim.org/docs/harmonic/ros_installation/#summary-of-compatible-ros-and-gazebo-combinations
# - https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#InstallGazebo
# - https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/
if { [ "$ROS_DISTRO" = "foxy" ] || [ "$ROS_DISTRO" = "humble" ]; } && [ "$TARGETARCH" = "amd64" ]; then
    echo "Installing Gazebo Classic (Fortress) for ROS $ROS_DISTRO..."
    apt-get install -y \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-gazebo-ros2-control
elif [ "$ROS_DISTRO" = "jazzy" ] && [ "$TARGETARCH" = "amd64" ]; then
    echo "Installing Gazebo Harmonic for ROS Jazzy..."
    apt-get install -y \
        ros-$ROS_DISTRO-ros-gz
else
    echo "Skipping Gazebo installation for ROS $ROS_DISTRO (not supported)"
fi

# Remove the apt cache
rm -rf /var/lib/apt/lists/*

echo "ROS $ROS_DISTRO installation completed successfully!"
