# Choose the architecture: arm or amd
ARG TARGET_PLATFORM=amd

# Base Image : https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm

# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd

# Install Gazebo
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

##########################################################################################

FROM $TARGET_PLATFORM

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

# Install ROS2 RVIZ
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install Kobuki dependencies
# The list of packages below was identified by executing the following command:
# sudo rosdep install --from-paths src --ignore-src -y
# Install these packages to avoid having to run `rosdep install` every time you build the workspace.
RUN apt-get update && apt-get install -y \
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