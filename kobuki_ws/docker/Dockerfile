# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64
# Base Image : https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH
ARG TARGETARCH

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

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

RUN if [ "$TARGETARCH" = "arm64" ]; then \
        # Fix the issue of Eigen
        # Reference: https://gitlab.com/libeigen/eigen/-/issues/2326
        sed -i 's/EIGEN_ALWAYS_INLINE eigen_packet_wrapper() {}/EIGEN_ALWAYS_INLINE eigen_packet_wrapper() = default;/' /usr/include/eigen3/Eigen/src/Core/GenericPacketMath.h; \
    fi

# Install Kobuki dependencies
# The list of packages below was identified by executing the following command:
# sudo rosdep install --from-paths src --ignore-src -y
# Install these packages to avoid having to run `rosdep install` every time you build the workspace.
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2 \
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
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    # Install navigation dependencies
    # Reference:
    # - https://docs.nav2.org/development_guides/build_docs/index.html
    # - https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-robot-localization \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo
# TODO: Run additional commands as non-root user here
COPY .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here

# Setup udev rules for Kobuki.
COPY udev_rules /home/$USERNAME/udev_rules
RUN /home/$USERNAME/udev_rules/install_udev_rules.sh

# Build the kobuki driver workspace
COPY --chown=$USER_UID kobuki_driver_ws /home/$USERNAME/kobuki_driver_ws
RUN cd /home/$USERNAME/kobuki_driver_ws \
    && rm -rf COLCON_IGNORE \
    && /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"

ENTRYPOINT []
CMD ["/bin/bash"]
