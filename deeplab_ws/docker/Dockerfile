# Base Image : https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch/tags
FROM nvcr.io/nvidia/pytorch:23.08-py3

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV SHELL=/bin/bash
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

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
    locales \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install ros2 humble
# Reference: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions

# Install pytorch dependencies
# Reference : https://forums.developer.nvidia.com/t/unable-to-import-pytorch/154016
RUN apt-get install -y libopenblas-base

COPY .bashrc /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
