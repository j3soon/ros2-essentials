# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64
# Base Image : https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

# Arguments for the default user
ARG USERNAME=user
ARG USER_UID=1000

# Keep downloaded packages for caching purposes
# Ref: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Upgrade packages
# Ref: https://pythonspeed.com/articles/security-updates-in-docker/
# Ref: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
# Ref: https://github.com/moby/buildkit/issues/1673#issuecomment-1264502398
# Ref: https://github.com/moby/buildkit/issues/1673#issuecomment-1987107404
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install sudo and create a user with sudo privileges
# Ref: https://stackoverflow.com/a/65434659
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    curl \
    git \
    htop \
    iputils-ping \
    nano \
    net-tools \
    tmux \
    tree \
    unzip \
    vim \
    wget \
    zip \
    && rm -rf /var/lib/apt/lists/*

# Install Python pip
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install custom tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    git-extras \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Gazebo packages for amd64
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    if [ "$TARGETARCH" = "amd64" ]; then \
        apt-get update && apt-get install -y \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-gazebo-ros2-control \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Install ROS2 RVIZ and other custom ROS2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# TODO: Add more commands here
# For example, to install additional packages, uncomment the following lines and add the package names
# RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
#     apt-get update && apt-get install -y \
#     $OTHER_PACKAGES \
#     && rm -rf /var/lib/apt/lists/*
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    # The packages below are used by the Husky repository.
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-urg-node \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-microstrain-inertial-driver \
    ros-$ROS_DISTRO-velodyne-driver \
    ros-$ROS_DISTRO-velodyne-pointcloud \
    ros-$ROS_DISTRO-robot-upstart \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-common \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo
# TODO: Run additional commands as non-root user here
COPY .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here

# Setup husky controller by the script.
# Build certain packages from source for arm64.
COPY script /home/$USERNAME/script
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update \
    && bash -ie /home/$USERNAME/script/install-clearpath-robot.sh \
    && sudo rm -rf /var/lib/apt/lists/*
# Note: The script need to be run as user, not root.
# Reference: https://github.com/clearpathrobotics/clearpath_computer_installer/tree/main
COPY clearpath_computer_installer /home/$USERNAME/clearpath_computer_installer
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update \
    && bash -ie /home/$USERNAME/clearpath_computer_installer/clearpath_computer_installer.sh \
    && sudo rm -rf /var/lib/apt/lists/*
# Setup udev rules.
COPY udev_rules /home/$USERNAME/udev_rules
RUN /home/$USERNAME/udev_rules/install_udev_rules.sh
# Generate robot configuration files.
RUN bash -ie /home/$USERNAME/script/husky-generate.sh

ENTRYPOINT []
CMD ["/bin/bash"]
