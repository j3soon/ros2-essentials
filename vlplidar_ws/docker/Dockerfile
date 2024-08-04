# Base Image: https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64
# Base Image: https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Reference: 
# - https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH

LABEL org.opencontainers.image.authors="assume0701@gmail.com"
LABEL description = \
        "An image for ROS2 VLP LiDAR."

ARG TARGETARCH
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

#############################################
# --- Basic setup ---

# Install necessary packages, Note: formation ref: template_ws
RUN apt-get update && apt-get install -y \
    sudo \
    vim \
    curl \
    tree \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 and Gazebo simulation packages when using `amd64` platform
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt update && apt install -y \
        ros-$ROS_DISTRO-turtlebot3-gazebo \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-rqt-robot-steering \
        && rm -rf /var/lib/apt/lists/*; \
        curl -sSL http://get.gazebosim.org | sh; \
    fi

# Install ROS2 velodyne package
RUN apt-get update && \
    apt-get install -y \
    ros-$ROS_DISTRO-turtlebot3 \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-velodyne && \
    rm -rf /var/lib/apt/lists/*

# # add user with default bash
RUN adduser --disabled-password --gecos '' --shell /bin/bash ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

COPY .bashrc /home/${USERNAME}/.bashrc

# Login with user
USER ${USERNAME}

CMD ["bash"]