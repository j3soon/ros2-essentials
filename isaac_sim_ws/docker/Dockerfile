FROM nvcr.io/nvidia/isaac-sim:4.1.0

# Ref: https://github.com/j3soon/docker-ros-humble-desktop-full/blob/master/Dockerfile

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-core/Dockerfile
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    # ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
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
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-base
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-base/Dockerfile
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
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
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-desktop
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/desktop/Dockerfile
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-desktop-full
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/desktop-full/Dockerfile
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Isaac Sim ROS 2 Optional Dependencies
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#running-native-ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-vision-msgs \
    ros-humble-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*

# Reset isaac sim docker image entrypoint
ENTRYPOINT []

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-core/Dockerfile
# setup entrypoint
# COPY ./thirdparty/ros_entrypoint.sh /
# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

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

COPY .bashrc /root/.bashrc
