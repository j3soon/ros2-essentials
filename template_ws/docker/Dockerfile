# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full

# Arguments for the default user
ARG USERNAME=user
ARG USER_UID=1000

# Upgrade packages
# Ref: https://pythonspeed.com/articles/security-updates-in-docker/
RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install sudo and create a user with sudo privileges
# Ref: https://stackoverflow.com/a/65434659
RUN apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Install common tools
RUN apt-get update && apt-get install -y \
    curl \
    git \
    htop \
    net-tools \
    tmux \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python pip
RUN apt-get update && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install custom tools
RUN apt-get update && apt-get install -y \
    git-extras \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 RVIZ and Gazebo
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# TODO: Add more commands here

COPY .bashrc /home/$USERNAME/.bashrc

USER $USERNAME
CMD ["/bin/bash"]
