################################################################################################
# Base Image : Ubuntu 22.04 Jammy Jellyfish
################################################################################################

FROM ubuntu:22.04

################################################################################################
# Label
################################################################################################

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"
LABEL description="This image is for ros1_bridge. (noetic <-> humble)"

################################################################################################
# Arguments
################################################################################################

ARG USERNAME=user
ARG USERID=$UID

################################################################################################
# Set environment variables
################################################################################################

ENV TERM=xterm-256color
ENV DEBIAN_FRONTEND=noninteractive
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV USERNAME=${USERNAME}
ENV PATH=$PATH:/home/${USERNAME}/.local/bin/

################################################################################################
# Install packages
################################################################################################

RUN apt update && \
    apt install -y \
    sudo \
    vim \
    curl \
    wget \
    git \
    git-extras \
    gnupg2 \
    net-tools

################################################################################################
# Set timezone (Taipei)
################################################################################################

RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

################################################################################################
# Add new user (Passwordless with sudo)
################################################################################################

RUN useradd -ms /bin/bash ${USERNAME} && \
    sudo adduser ${USERNAME} sudo && \
    echo "${USERNAME}:ubuntu" | chpasswd && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers 
USER ${USERNAME}

################################################################################################
# Install ROS Humble from source
# Reference : 
# - https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
# - https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
################################################################################################

# Add the ROS 2 apt repository
RUN sudo apt install -y \
    software-properties-common \
    lsb-release && \
    sudo add-apt-repository universe && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
RUN sudo apt update && \
    sudo apt install -y \
    build-essential \
    cmake \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep \
    python3-setuptools \
    ros-dev-tools

# Create ros2_humble workspace
RUN mkdir -p /home/${USERNAME}/ros2_humble/src
WORKDIR /home/${USERNAME}/ros2_humble

# Install colcon from PyPI, rather than apt packages
RUN pip3 install -U colcon-common-extensions vcstool

# clone all repos
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install dependencies
RUN sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build the code in the workspace
RUN colcon build --symlink-install

# Remove the packages.ros.org from sources.list and conflicts packages
RUN sudo rm -rf /etc/apt/sources.list.d/ros2.list && \
    sudo apt remove -y \
    python3-catkin-pkg \
    python3-catkin-pkg-modules

# Install ROS 1 from Ubuntu packages
RUN sudo apt update && \
    sudo apt install -y ros-core-dev

################################################################################################
# Build ros1_bridge from source
################################################################################################

RUN git clone https://github.com/ros2/ros1_bridge src/ros1_bridge
RUN /bin/bash -c "source install/local_setup.bash && colcon build --packages-select ros1_bridge --cmake-force-configure"

################################################################################################
# Final settings
################################################################################################

COPY ./start_bridge.sh /home/${USERNAME}/start_bridge.sh 
WORKDIR /home/${USERNAME}
SHELL ["/bin/bash", "-c"]
CMD ["./start_bridge.sh"]
