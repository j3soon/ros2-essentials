# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64
# Base Image : https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH

# Arguments for the default user
ARG USERNAME=user
ARG USER_UID=1000

# Upgrade packages
# Ref: https://pythonspeed.com/articles/security-updates-in-docker/
# Ref: https://docs.docker.com/build/cache/#use-the-dedicated-run-cache
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install sudo and create a user with sudo privileges
# Ref: https://stackoverflow.com/a/65434659
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt \
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
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install custom tools
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    git-extras \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Gazebo packages for amd64
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt-get update && apt-get install -y \
        ros-$ROS_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-gazebo-ros2-control \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# Install ROS2 RVIZ and other custom ROS2 packages
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# TODO: Add more commands here

USER $USERNAME
COPY .bashrc /home/$USERNAME/.bashrc
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo
ENTRYPOINT []
CMD ["/bin/bash"]
