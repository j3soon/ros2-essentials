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

# Isaac Sim version configuration
ARG ISAAC_SIM_VERSION=4.5.0
# Ref: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PATH="/home/$USERNAME/isaacsim"
# Copy and run Isaac Sim installation script
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_sim.sh /tmp/install_isaac_sim.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private \
    ISAAC_SIM_VERSION=$ISAAC_SIM_VERSION /tmp/install_isaac_sim.sh && rm /tmp/install_isaac_sim.sh
# Set Isaac Sim environment variables
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#running-isaac-sim
# Ref: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/blob/e3c09375c2d110b39c3fab3611352870aa3ce6ee/Dockerfile.2023.1.0-ubuntu22.04#L49-L53
ENV OMNI_USER=admin
ENV OMNI_PASS=admin
ENV OMNI_KIT_ACCEPT_EULA=YES
# Ref: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Create Isaac Sim directories with correct ownership to avoid permission issues after volume mount
RUN sudo mkdir -p /isaac-sim && sudo chown $USERNAME:$USERNAME /isaac-sim
RUN mkdir -p /isaac-sim/kit/cache \
    && mkdir -p /home/$USERNAME/.cache/ov \
    && mkdir -p /home/$USERNAME/.local/lib/python3.10/site-packages/omni/cache \
    && mkdir -p /home/$USERNAME/.cache/pip \
    && mkdir -p /home/$USERNAME/.cache/nvidia/GLCache \
    && mkdir -p /home/$USERNAME/.nv/ComputeCache \
    && mkdir -p /home/$USERNAME/.nvidia-omniverse/logs \
    && mkdir -p /home/$USERNAME/.local/lib/python3.10/site-packages/omni/logs \
    && mkdir -p /home/$USERNAME/.local/share/ov/data \
    && mkdir -p /home/$USERNAME/.local/lib/python3.10/site-packages/omni/data \
    && mkdir -p /home/$USERNAME/Documents

# Set TERM to prevent Isaac Lab error during docker build:
#    'ansi+tabs': unknown terminal type.
ENV TERM=xterm-256color
# Isaac Lab version configuration
ARG ISAAC_LAB_VERSION=2.1.0
# Ref: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/policy_deployment/00_hover/hover_policy.html
ENV ISAACLAB_PATH="/home/$USERNAME/IsaacLab"
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_lab.sh /tmp/install_isaac_lab.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private \
    ISAAC_LAB_VERSION=$ISAAC_LAB_VERSION /tmp/install_isaac_lab.sh && rm /tmp/install_isaac_lab.sh

# Isaac ROS version configuration
ARG ISAAC_ROS=YES
ENV ISAAC_ROS_WS="/home/$USERNAME/workspaces/isaac_ros-dev/"
# Copy and run Isaac ROS installation script
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_ros.sh /tmp/install_isaac_ros.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    ISAAC_ROS=$ISAAC_ROS /tmp/install_isaac_ros.sh && rm /tmp/install_isaac_ros.sh
# Ref: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

# Install custom tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
    git-extras \
    && sudo rm -rf /var/lib/apt/lists/*

# TODO: Add more commands here
COPY --chown=$USERNAME:$USERNAME \
     .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here
ENTRYPOINT []
CMD ["/bin/bash"]
