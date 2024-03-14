# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

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

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rviz2 \
    #
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
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-common \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-imu-tools \
    && sudo rm -rf /var/lib/apt/lists/*

COPY .bashrc /home/$USERNAME/.bashrc 

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

# Setup husky controller by the script.
# Note: The script need to be run as user, not root.
# Reference: https://github.com/clearpathrobotics/clearpath_computer_installer/tree/main
COPY clearpath_computer_installer /home/$USERNAME/clearpath_computer_installer
RUN bash /home/$USERNAME/clearpath_computer_installer/clearpath_computer_installer.sh