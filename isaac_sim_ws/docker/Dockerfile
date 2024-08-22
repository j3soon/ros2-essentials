# Base Image: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim/tags
FROM nvcr.io/nvidia/isaac-sim:4.1.0 AS amd64
# Base Image: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim/tags
FROM nvcr.io/nvidia/isaac-sim:4.1.0 AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH
ARG TARGETARCH

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

# Ref: https://github.com/j3soon/docker-ros-humble-desktop-full/blob/master/Dockerfile

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-core/Dockerfile
# setup timezone
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    echo 'Etc/UTC' > /etc/timezone && \
    # ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
# install packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*
# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble
# install ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-base
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-base/Dockerfile
# install bootstrap tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO
# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
# install ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-desktop
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/desktop/Dockerfile
# install ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-desktop-full
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/desktop-full/Dockerfile
# install ros2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop-full=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-core/Dockerfile
# setup entrypoint
# COPY ./thirdparty/ros_entrypoint.sh /
# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]

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
# Isaac Sim ROS 2 Optional Dependencies
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#running-native-ros
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo
# TODO: Run additional commands as non-root user here
COPY .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here
ENTRYPOINT []
CMD ["/bin/bash"]
