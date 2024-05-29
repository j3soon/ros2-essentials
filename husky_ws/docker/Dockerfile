# Base Image : https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64
# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64

FROM $TARGETARCH

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

RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt-get update && apt-get install -y \
        && ros-$ROS_DISTRO-gazebo-ros-pkgs \
        && ros-$ROS_DISTRO-gazebo-ros2-control \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
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
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-realsense2-description \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-common \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    && sudo rm -rf /var/lib/apt/lists/*

COPY .bashrc /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

# Setup husky controller by the script.
# Build certain packages from source for arm64.
COPY script /home/$USERNAME/script
RUN sudo apt-get update \
    && bash -ie /home/$USERNAME/script/install-clearpath-robot.sh \
    && sudo rm -rf /var/lib/apt/lists/*
# Note: The script need to be run as user, not root.
# Reference: https://github.com/clearpathrobotics/clearpath_computer_installer/tree/main
COPY clearpath_computer_installer /home/$USERNAME/clearpath_computer_installer
RUN sudo apt-get update \
    && bash -ie /home/$USERNAME/clearpath_computer_installer/clearpath_computer_installer.sh \
    && sudo rm -rf /var/lib/apt/lists/*
# Setup udev rules.
COPY udev_rules /home/$USERNAME/udev_rules
RUN /home/$USERNAME/udev_rules/install_udev_rules.sh
# Generate robot configuration files.
RUN bash -ie /home/$USERNAME/script/husky-generate.sh
