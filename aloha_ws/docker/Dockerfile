# Base Image: https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full AS amd64
# Base Image: https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:humble AS arm64

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

# Install GUI debugging tools
# - `x11-apps` and `x11-utils` for `xeyes` and `xdpyinfo`
#   Ref: https://packages.debian.org/sid/x11-apps
#   Ref: https://packages.debian.org/sid/x11-utils
# - `mesa-utils` for `glxgears` and `glxinfo`
#   Ref: https://wiki.debian.org/Mesa
# - `vulkan-tools` for `vkcube` and `vulkaninfo`
#   Ref: https://docs.vulkan.org/tutorial/latest/02_Development_environment.html#_vulkan_packages
#   Ref: https://gitlab.com/nvidia/container-images/vulkan/-/blob/master/docker/Dockerfile.ubuntu
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    x11-apps x11-utils \
    mesa-utils \
    libgl1 vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# Setup the required capabilities for the container runtime
# Ref: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html#driver-capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Install Vulkan config files
# Ref: https://gitlab.com/nvidia/container-images/vulkan
# Ref: https://github.com/j3soon/docker-vulkan-runtime
RUN cat > /etc/vulkan/icd.d/nvidia_icd.json <<EOF
{
    "file_format_version" : "1.0.0",
    "ICD": {
        "library_path": "libGLX_nvidia.so.0",
        "api_version" : "1.3.194"
    }
}
EOF
RUN mkdir -p /usr/share/glvnd/egl_vendor.d && \
    cat > /usr/share/glvnd/egl_vendor.d/10_nvidia.json <<EOF
{
    "file_format_version" : "1.0.0",
    "ICD" : {
        "library_path" : "libEGL_nvidia.so.0"
    }
}
EOF
RUN cat > /etc/vulkan/implicit_layer.d/nvidia_layers.json <<EOF
{
    "file_format_version" : "1.0.0",
    "layer": {
        "name": "VK_LAYER_NV_optimus",
        "type": "INSTANCE",
        "library_path": "libGLX_nvidia.so.0",
        "api_version" : "1.3.194",
        "implementation_version" : "1",
        "description" : "NVIDIA Optimus layer",
        "functions": {
            "vkGetInstanceProcAddr": "vk_optimusGetInstanceProcAddr",
            "vkGetDeviceProcAddr": "vk_optimusGetDeviceProcAddr"
        },
        "enable_environment": {
            "__NV_PRIME_RENDER_OFFLOAD": "1"
        },
        "disable_environment": {
            "DISABLE_LAYER_NV_OPTIMUS_1": ""
        }
    }
}
EOF

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

# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo

# Install `libxrandr2` to support Isaac Sim WebRTC streaming
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    if [ "$TARGETARCH" = "amd64" ]; then \
        sudo apt-get update && sudo apt-get install -y \
        libxrandr2 \
        && sudo rm -rf /var/lib/apt/lists/*; \
    fi
# Install Isaac Sim (requires Python 3.10)
# Note that installing Isaac Sim with pip is experimental, keep this in mind when unexpected error occurs
# TODO: Remove the note above when it is no longer experimental
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#installation-using-pip
RUN --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private \
    if [ "$TARGETARCH" = "amd64" ]; then \
        python3 -V | grep "Python 3.10" \
        && pip install isaacsim==4.2.0.2 --extra-index-url https://pypi.nvidia.com \
        && pip install isaacsim-extscache-physics==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 --extra-index-url https://pypi.nvidia.com; \
    fi

# Fix SciPy (in base image) incompatibility with NumPy version (in Isaac Sim) `numpy==1.26.0`, only for amd64
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        pip install scipy==1.14.1 numpy==1.26.0; \
    fi

# Create isaac sim cache directory with correct ownership to avoid permission issues after volume mount, only for amd64
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        sudo mkdir -p /isaac-sim/kit/cache \
        && sudo chown -R $USERNAME:$USERNAME /isaac-sim/kit/cache; \
    fi
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        mkdir -p /home/$USERNAME/.cache/ov \
        && mkdir -p /home/$USERNAME/.cache/pip \
        && mkdir -p /home/$USERNAME/.cache/nvidia/GLCache \
        && mkdir -p /home/$USERNAME/.nv/ComputeCache \
        && mkdir -p /home/$USERNAME/.nvidia-omniverse/logs \
        && mkdir -p /home/$USERNAME/.local/share/ov/data \
        && mkdir -p /home/$USERNAME/Documents; \
    fi

# Set Isaac Sim environment variables
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#running-isaac-sim
# Ref: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/blob/e3c09375c2d110b39c3fab3611352870aa3ce6ee/Dockerfile.2023.1.0-ubuntu22.04#L49-L53
ENV OMNI_USER=admin
ENV OMNI_PASS=admin
ENV OMNI_KIT_ACCEPT_EULA=YES

# Install custom tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
    git-extras \
    && sudo rm -rf /var/lib/apt/lists/*

# TODO: Add more commands here

# Install udevadm for later use
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
    udev \
    && sudo rm -rf /var/lib/apt/lists/*

# Install Aloha Interbotix dependencies
# Ref: https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#amd64-architecture
# Ref: https://github.com/Interbotix/interbotix_ros_manipulators/blob/humble/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh
# Install ros packages based on Interbotix rosdeps to speed up build
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
    ros-$ROS_DISTRO-moveit-msgs \
    ros-$ROS_DISTRO-moveit-ros-planning-interface \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-dynamixel-sdk \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-ros2controlcli \
    ros-$ROS_DISTRO-graph-msgs \
    ros-$ROS_DISTRO-moveit-common \
    ros-$ROS_DISTRO-moveit-core \
    ros-$ROS_DISTRO-moveit-ros-planning \
    ros-$ROS_DISTRO-rviz-visual-tools \
    ros-$ROS_DISTRO-moveit-ros-move-group \
    ros-$ROS_DISTRO-moveit-simple-controller-manager \
    ros-$ROS_DISTRO-moveit-kinematics \
    ros-$ROS_DISTRO-moveit-planners-ompl \
    ros-$ROS_DISTRO-moveit-ros-visualization \
    ros-$ROS_DISTRO-moveit-setup-assistant \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-topic-based-ros2-control \
    && sudo rm -rf /var/lib/apt/lists/*
COPY --chown=$USERNAME:$USERNAME \
     script/xsarm_amd64_install.sh /home/$USERNAME
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    bash -ie ~/xsarm_amd64_install.sh -d humble -n \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf $HOME/.ros/rosdep/sources.cache
COPY udev_rules/99-interbotix-udev.rules /etc/udev/rules.d/

COPY --chown=$USERNAME:$USERNAME \
     .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here
ENTRYPOINT []
CMD ["/bin/bash"]
