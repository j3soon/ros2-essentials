################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: Ubuntu 22.04 Jammy Jellyfish
#       - https://hub.docker.com/_/ubuntu/tags?page=1&name=22.04
################################################################################################

FROM ubuntu:22.04 AS base

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

# Reference: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#set-locale
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# Add user
RUN useradd -ms /bin/bash user

################################################################################################
# - Build stage
#   - In this stage, I will build ROS Humble and ros1-bridge from source.
#   - Reference: 
#     - https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
#     - https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
################################################################################################

FROM base AS build

# Install the required packages for the following command.
RUN apt-get update && \
    apt-get install -y \
    curl \
    locales \
    software-properties-common \
    lsb-release

# Set locale.
# Reference: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#set-locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the ROS 2 apt repository.
# Reference: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#add-the-ros-2-apt-repository
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools.
# Note that we declare noninteractive mode to avoid apt asking for user input.
# Additionally, install colcon from PyPI, rather than using apt packages.
# Reference: https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html#install-development-tools-and-ros-tools
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    git \
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
RUN python3 -m pip install -U colcon-common-extensions vcstool

# Create ros2_humble workspace, and clone all repositories from the list.
# Reference: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#get-ros-2-code
RUN mkdir -p /ros2_humble/src
WORKDIR /ros2_humble
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install dependencies.
# Reference: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#install-dependencies-using-rosdep
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build the ros2_humble workspace. (This will take a few minutes)
# Note that we are using colcon build without the flag --symlink-install.
# This is because we will copy the built files from the build stage to the release stage.
# If we use the flag --symlink-install, the release stage will not be able to find the built files.
# Reference: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#build-the-code-in-the-workspace
RUN colcon build

# Delete packages.ros.org from sources.list and remove conflicts packages manually.
# Removing those conflicts packages is necessary for install ros-core-dev.
# Here is the dicussion about this issue: https://github.com/j3soon/ros2-essentials/pull/9#discussion_r1375303858
# Reference: 
# - https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html#ros-2-via-debian-packages
# - https://github.com/osrf/docker_images/issues/635#issuecomment-1217505552
RUN rm -rf /etc/apt/sources.list.d/ros2.list && \
    apt-get remove -y \
    python3-catkin-pkg \
    python3-catkin-pkg-modules

# Install ROS 1 from Ubuntu packages.
# Reference: https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html#install-ros-1-from-ubuntu-packages
RUN apt-get update && \
    apt-get install -y ros-core-dev

# Build ros1_bridge from source.
# Reference: https://github.com/ros2/ros1_bridge/tree/3d5328dc21564d2130b4ded30afe5cd1c41cf033#building-the-bridge-from-source
RUN git clone https://github.com/ros2/ros1_bridge src/ros1_bridge
RUN source install/setup.bash && \
    colcon build --packages-select ros1_bridge --cmake-force-configure

# Other configurations
COPY ./start-bridge.sh /start-bridge.sh 
USER user
WORKDIR /
CMD ["/bin/bash"]

################################################################################################
# - Release stage
#   - In this stage, I will copy the built files from the build stage.
#     This is useful for reducing the size of the image.
#   - Reference:
#     - https://docs.docker.com/build/building/multi-stage/
################################################################################################

FROM base AS release

# Install ROS core and packages with their dependencies. 
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-core-dev \
    python3-packaging \
    libspdlog-dev

# Copy files from host and build stage
COPY --from=build /ros2_humble/install /ros2_humble/install

# Other configurations
COPY ./start-bridge.sh /start-bridge.sh 
USER user
WORKDIR /
CMD ["./start-bridge.sh"]