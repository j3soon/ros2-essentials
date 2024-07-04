# Base Image : https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64
# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Reference: 
# - https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

ARG TARGETARCH
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Install common tools
RUN apt-get update && apt-get install -y \
    curl \
    git \
    git-extras \
    htop \
    net-tools \
    tmux \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 RVIZ and Gazebo
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt-get update && apt-get install -y \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-gazebo-ros2-control \
        && rm -rf /var/lib/apt/lists/*; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        # Fix the issue of Eigen
        # Reference:
        # - https://gitlab.com/libeigen/eigen/-/issues/2326
        sed -i 's/EIGEN_ALWAYS_INLINE eigen_packet_wrapper() {}/EIGEN_ALWAYS_INLINE eigen_packet_wrapper() = default;/' /usr/include/eigen3/Eigen/src/Core/GenericPacketMath.h; \
    fi

# Install Kobuki dependencies
# The list of packages below was identified by executing the following command:
# sudo rosdep install --from-paths src --ignore-src -y
# Install these packages to avoid having to run `rosdep install` every time you build the workspace.
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ros-testing \
    ros-$ROS_DISTRO-kobuki-velocity-smoother \
    ros-$ROS_DISTRO-diagnostics \
    ros-$ROS_DISTRO-ecl-license \
    ros-$ROS_DISTRO-ecl-build \
    ros-$ROS_DISTRO-ecl-tools \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-diff-drive-controller \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    #
    # Install navigation dependencies
    # Reference:
    # - https://docs.nav2.org/development_guides/build_docs/index.html
    # - https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-robot-localization \
    && rm -rf /var/lib/apt/lists/*

COPY .bashrc /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

# Setup udev rules for Kobuki.
COPY udev_rules /home/$USERNAME/udev_rules
RUN /home/$USERNAME/udev_rules/install_udev_rules.sh

# Build the kobuki driver workspace
COPY --chown=$USER_UID:$USER_GID kobuki_driver_ws /home/$USERNAME/kobuki_driver_ws
RUN cd /home/$USERNAME/kobuki_driver_ws \
    && rm -rf COLCON_IGNORE \
    && /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"