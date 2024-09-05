# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full

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

# Install Aloha Interbotix dependencies
# Ref: https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#amd64-architecture
# Ref: https://github.com/Interbotix/interbotix_ros_manipulators/blob/humble/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh
RUN apt-get update && apt-get install -y curl \
    && curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/d07c17cc0f868bf1f59ee8d8fa844e273f4699ff/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh \
    && chmod +x xsarm_amd64_install.sh \
    && ./xsarm_amd64_install.sh -d humble \
    && rm -rf /var/lib/apt/lists/*

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
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

COPY .bashrc /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
