# Base Image: https://hub.docker.com/_/ubuntu/tags?name=22.04
FROM ubuntu:22.04 AS amd64
# Base Image: https://hub.docker.com/_/ubuntu/tags?name=22.04
FROM ubuntu:22.04 AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH
ARG TARGETARCH

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

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    curl \
    git \
    git-lfs \
    htop \
    iputils-ping \
    jq \
    nano \
    net-tools \
    netcat-traditional \
    nmap \
    p7zip-full \
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

# ROS version configuration
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=$ROS_DISTRO
# Copy and run ROS installation script
COPY modules/install_ros.sh /tmp/install_ros.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_ros.sh && rm /tmp/install_ros.sh
# Set the CycloneDDS configuration file
ENV CYCLONEDDS_URI=/home/user/cyclonedds.xml

# Setup the required capabilities for the container runtime
# Ref: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html#driver-capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Install GUI debugging tools and configure Vulkan
COPY modules/install_x11_opengl_vulkan.sh /tmp/install_x11_opengl_vulkan.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_x11_opengl_vulkan.sh && rm /tmp/install_x11_opengl_vulkan.sh

# Install CUDA Toolkit (not installed by default)
ARG CUDA_TOOLKIT_VERSION=""
# Copy and run CUDA Toolkit installation script
COPY modules/install_cuda_toolkit.sh /tmp/install_cuda_toolkit.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_cuda_toolkit.sh && rm /tmp/install_cuda_toolkit.sh
# Ref: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#post-installation-actions
ENV PATH="${PATH}:/usr/local/cuda-${CUDA_TOOLKIT_VERSION}/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/cuda-${CUDA_TOOLKIT_VERSION}/lib64"

# Arguments for the default user
ARG USERNAME=user
ARG USER_UID=1000

# Install sudo and create a user with sudo privileges
# Ref: https://stackoverflow.com/a/65434659
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Create cache directories with correct ownership to avoid permission issues after volume mount
RUN mkdir -p /home/$USERNAME/.cache/pip
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo

# Isaac Sim version configuration
ARG ISAAC_SIM_VERSION=5.0.0
# Ref: https://isaac-sim.github.io/IsaacLab/v2.2.1/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PATH="/home/$USERNAME/isaacsim"
# Copy and run Isaac Sim installation script
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_sim.sh /tmp/install_isaac_sim.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    /tmp/install_isaac_sim.sh && rm /tmp/install_isaac_sim.sh
# Set Isaac Sim environment variables
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#running-isaac-sim
# Ref: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/blob/e3c09375c2d110b39c3fab3611352870aa3ce6ee/Dockerfile.2023.1.0-ubuntu22.04#L49-L53
ENV OMNI_USER=admin
ENV OMNI_PASS=admin
ENV OMNI_KIT_ACCEPT_EULA=YES
# Ref: https://isaac-sim.github.io/IsaacLab/v2.2.1/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Set TERM to prevent Isaac Lab error during docker build:
#    'ansi+tabs': unknown terminal type.
ENV TERM=xterm-256color
# Isaac Lab version configuration
ARG ISAAC_LAB_VERSION=2.2.1
# Ref: https://isaac-sim.github.io/IsaacLab/v2.2.1/source/policy_deployment/00_hover/hover_policy.html
ENV ISAACLAB_PATH="/home/$USERNAME/IsaacLab"
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_lab.sh /tmp/install_isaac_lab.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    /tmp/install_isaac_lab.sh && rm /tmp/install_isaac_lab.sh

# Isaac ROS version configuration
ARG ISAAC_ROS=""
ENV ISAAC_ROS_WS="/home/$USERNAME/workspaces/isaac_ros-dev/"
# Copy and run Isaac ROS installation script
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_ros.sh /tmp/install_isaac_ros.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_isaac_ros.sh && rm /tmp/install_isaac_ros.sh
# Ref: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

# Install custom tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
    git-extras \
    && sudo rm -rf /var/lib/apt/lists/*

# Cartographer configuration
ARG CARTOGRAPHER=NO
COPY --chown=$USERNAME:$USERNAME \
     modules/install_cartographer.sh /tmp/install_cartographer.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_cartographer.sh && rm /tmp/install_cartographer.sh

# TODO: Add more commands here
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
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
    && sudo rm -rf /var/lib/apt/lists/*

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

COPY --chown=$USERNAME:$USERNAME \
     .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here

LABEL org.opencontainers.image.authors="assume0701@gmail.com"
ENTRYPOINT []
CMD ["/bin/bash"]
