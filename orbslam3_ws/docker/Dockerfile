# Base Image: https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64
# Base Image: https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH
ARG TARGETARCH

LABEL org.opencontainers.image.authors="assume0701@gmail.com"

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
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    # ORB-SLAM dependencies
    libopencv-dev \
    opencv-data \
    libglew-dev \
    ffmpeg \
    libboost-system-dev \
    libeigen3-dev \
    # ROS dependencies
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-message-filters \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo
# TODO: Run additional commands as non-root user here

# Build Pangolin from source at commit: https://github.com/stevenlovegrove/Pangolin/tree/d484494645cb7361374ac0ef6b27e9ee6feffbd7
# TODO(@j3soon): Simplify the git commands here
RUN mkdir -p /home/${USERNAME}/Pangolin/build && \
    cd /home/${USERNAME}/Pangolin && \
    git init && \
    git remote add origin https://github.com/stevenlovegrove/Pangolin.git && \
    git pull origin d484494645cb7361374ac0ef6b27e9ee6feffbd7 && \
    git rm -r --cached .
RUN cd /home/${USERNAME}/Pangolin/build && \
    cmake .. && \
    cmake --build .

# Build ORB-SLAM3 from source at commit: https://github.com/UZ-SLAMLab/ORB_SLAM3/commit/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
# TODO(@j3soon): Simplify the git commands here
RUN mkdir -p /home/${USERNAME}/ORB_SLAM3 && \
    cd /home/${USERNAME}/ORB_SLAM3 && \
    git init && \
    git remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git && \
    git pull origin 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4 && \
    git rm -r --cached .
# Change to c++14 for resolving compile error
# Ref : https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/387
# Ref : https://github.com/Soldann/MORB_SLAM/issues/3
RUN sed -i 's/++11/++14/g' /home/${USERNAME}/ORB_SLAM3/CMakeLists.txt && \
    sed -i 's/++11/++14/g' /home/${USERNAME}/ORB_SLAM3/Thirdparty/g2o/CMakeLists.txt && \
    sed -i 's/++11/++14/g' /home/${USERNAME}/ORB_SLAM3/Thirdparty/Sophus/CMakeLists.txt
RUN cd /home/${USERNAME}/ORB_SLAM3 && \
    ./build.sh
RUN cd /home/${USERNAME}/ORB_SLAM3/Thirdparty/Sophus/build && \
    sudo make install

# Build ORB-SLAM3 for ROS2
RUN mkdir -p /home/${USERNAME}/test_ws/src
WORKDIR /home/${USERNAME}/test_ws
RUN colcon build
RUN mkdir -p src/orbslam3_ros2 && \
    cd src/orbslam3_ros2 && \
    git init && \
    git remote add origin https://github.com/zang09/ORB_SLAM3_ROS2.git && \
    git pull origin 00c54335ccc010d74c1e24e336aa817604124947 && \
    git rm -r --cached .
# Fix path error and colcon build error
# Ref : https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970
RUN sed -i 's/foxy\/lib\/python3.8/humble\/lib\/python3.10/g' src/orbslam3_ros2/CMakeLists.txt && \
    sed -i 's/Pangolin)/Pangolin OpenCV)/g' src/orbslam3_ros2/CMakeLists.txt && \
    sed -i 's/Install\/ORB_SLAM\/ORB_SLAM3/ORB_SLAM3/g' src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake
RUN rosdep update && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select orbslam3

COPY .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here
ENTRYPOINT []
CMD ["/bin/bash"]
