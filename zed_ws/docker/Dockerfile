# Base Image : https://hub.docker.com/r/stereolabs/zed/tags?page=1&name=22.04
FROM stereolabs/zed:4.0-devel-cuda11.8-ubuntu22.04

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
    locales \
    software-properties-common \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2-humble
# Reference: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
ENV ROS_DISTRO=humble
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Init rosdep
RUN rosdep init \
    && rosdep update

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    # RVIZ and Gazebo
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rviz2 \
    # ZED dependencies
    # From the command: rosdep install --from-paths src --ignore-src -r -y
    ros-$ROS_DISTRO-ament-cmake-clang-format \
    ros-$ROS_DISTRO-grid-map-rviz-plugin \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Change the ownership of the ZED folder
# Reference: https://answers.ros.org/question/293850
RUN chown -R $USERNAME:$USERNAME /usr/local/zed

COPY .bashrc /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
