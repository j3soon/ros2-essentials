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
    netcat-openbsd \
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

# Create user bin directories specified in `~/.profile`
# Together with .bashrc, this allows invoking executables in the same shell after installing to either location
RUN mkdir /home/$USERNAME/bin
RUN mkdir -p /home/$USERNAME/.local/bin

# Create cache directories with correct ownership to avoid permission issues after volume mount
RUN mkdir -p /home/$USERNAME/.cache/pip
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo

# Isaac Sim version configuration
ARG ISAAC_SIM_VERSION=5.1.0
# Ref: https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/binaries_installation.html
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
# Ref: https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Set TERM to prevent Isaac Lab error during docker build:
#    'ansi+tabs': unknown terminal type.
ENV TERM=xterm-256color
# Isaac Lab version configuration
ARG ISAAC_LAB_VERSION=2.3.2
# Ref: https://isaac-sim.github.io/IsaacLab/v2.3.2/source/policy_deployment/00_hover/hover_policy.html
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
ARG CARTOGRAPHER=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_cartographer.sh /tmp/install_cartographer.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_cartographer.sh && rm /tmp/install_cartographer.sh

# rtabmap configuration
ARG RTABMAP=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_rtabmap.sh /tmp/install_rtabmap.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_rtabmap.sh && rm /tmp/install_rtabmap.sh

# realsense configuration
ARG REALSENSE=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_realsense.sh /tmp/install_realsense.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_realsense.sh && rm /tmp/install_realsense.sh

# TODO: Add more commands here
# Ref: https://huggingface.co/docs/lerobot/installation
# Install ffmpeg 7 from ubuntuhandbook PPA
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo add-apt-repository -y ppa:ubuntuhandbook1/ffmpeg7 \
    && sudo apt-get update && sudo apt-get install -y ffmpeg \
    && sudo rm -rf /var/lib/apt/lists/*
# # Install `spd-say`, which is used in many lerobot command for text-to-speech signaling.
# RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
#     sudo apt-get update && sudo apt-get install -y speech-dispatcher \
#     && sudo rm -rf /var/lib/apt/lists/*
# Install lerobot
# Note: `colcon-core 0.20.1` requires `setuptools<80`
#        numpy 2.x requires a newer version of scipy, therefore the upgrade
RUN --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    python3 -m pip install "lerobot[feetech]==0.4.4" && \
    python3 -m pip install "setuptools<80" && \
    python3 -m pip install -U scipy

# Install LeIsaac (official instructions assume Isaac Sim 5.1.0 and Isaac Lab 2.3.0)
# Ref: https://lightwheelai.github.io/leisaac/docs/getting_started/installation/
# Ref: https://huggingface.co/docs/lerobot/main/en/envhub_leisaac
RUN --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    cd /home/$USERNAME && \
    git clone https://github.com/LightwheelAI/leisaac.git && \
    cd /home/$USERNAME/leisaac && \
    git checkout 241779b8439593d4bec4df5e3182a97f205814b5 && \
    /home/$USERNAME/IsaacLab/isaaclab.sh -p -m pip install "source/leisaac"

# Download LeIsaac Assets
# Ref: https://lightwheelai.github.io/leisaac/docs/getting_started/installation/#2-asset-preparation
RUN mkdir -p /home/$USERNAME/leisaac/assets/robots && \
    mkdir -p /home/$USERNAME/leisaac/assets/scenes && \
    cd /home/$USERNAME/leisaac/assets && \
    wget -q https://github.com/LightwheelAI/leisaac/releases/download/v0.1.0/so101_follower.usd \
        -O robots/so101_follower.usd && \
    wget -q https://github.com/LightwheelAI/leisaac/releases/download/v0.1.0/kitchen_with_orange.zip \
        -O /tmp/kitchen_with_orange.zip && \
    unzip -q /tmp/kitchen_with_orange.zip -d scenes && \
    rm -f /tmp/kitchen_with_orange.zip
    # There are several other scenes (4 at the time of writing) that can be downloaded, but we only download one for now to save space.

# Install Lerobot Dataset Visualizer and its dependencies
# Ref: https://bun.sh/docs/installation
# Ref: https://github.com/huggingface/lerobot-dataset-visualizer
RUN curl -fsSL https://bun.sh/install | bash
RUN git clone -b feat/private_repo_viz https://github.com/huggingface/lerobot-dataset-visualizer.git /home/$USERNAME/lerobot-dataset-visualizer
RUN cd /home/$USERNAME/lerobot-dataset-visualizer && /home/$USERNAME/.bun/bin/bun install
COPY --chown=$USERNAME:$USERNAME \
     .bashrc /home/$USERNAME/.bashrc
# TODO: Copy additional files here
# NVIDIA OpenUSD Tools configuration
ARG NV_OPENUSD=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_nv_openusd.sh /tmp/install_nv_openusd.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_nv_openusd.sh && rm /tmp/install_nv_openusd.sh

# Newton Tools configuration
ARG NEWTON_TOOLS=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_newton_tools.sh /tmp/install_newton_tools.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    /tmp/install_newton_tools.sh && rm /tmp/install_newton_tools.sh

# Claude Code CLI configuration (installed last since the installation is fast)
ARG CLAUDE_CODE=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_claude_code.sh /tmp/install_claude_code.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_claude_code.sh && rm /tmp/install_claude_code.sh
# Codex CLI configuration
ARG CODEX=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_codex.sh /tmp/install_codex.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_codex.sh && rm /tmp/install_codex.sh
# OpenCode CLI configuration
ARG OPENCODE=""
COPY --chown=$USERNAME:$USERNAME \
     modules/install_opencode.sh /tmp/install_opencode.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_opencode.sh && rm /tmp/install_opencode.sh
# TODO: Install more optional development tools here
ENTRYPOINT []
CMD ["/bin/bash"]
