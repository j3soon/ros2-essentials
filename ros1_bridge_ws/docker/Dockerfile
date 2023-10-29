################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: Ubuntu 22.04 Jammy Jellyfish
#       - https://hub.docker.com/_/ubuntu/tags?page=1&name=22.04
################################################################################################

FROM ubuntu:22.04 AS base

LABEL org.opencontainers.image.authors="yuzhong1214@gmail.com"

ENV TERM=xterm-256color
ENV DEBIAN_FRONTEND=noninteractive
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN useradd -ms /bin/bash user
COPY ./start-bridge.sh /start-bridge.sh 

################################################################################################
# - Build stage
#   - In this stage, I will build ROS Humble and ros1-bridge from source.
#   - Reference: 
#     - https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
#     - https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
################################################################################################

FROM base AS build

# Install packages.
RUN apt-get update && \
    apt-get install -y \
    curl \
    git \
    locales \
    software-properties-common \
    lsb-release

# Set locale.
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add the ROS 2 apt repository.
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools.
RUN apt-get update && \
    apt-get install -y \
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

# Create ros2_humble workspace.
RUN mkdir -p /ros2_humble/src
WORKDIR /ros2_humble

# Install colcon from PyPI, rather than apt packages.
RUN pip3 install -U colcon-common-extensions vcstool

# Clone all repositories from the list.
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install dependencies.
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build the ros2_humble workspace. (This will take a few minutes)
RUN colcon build

# Delete packages.ros.org from sources.list and remove conflicts packages.
RUN rm -rf /etc/apt/sources.list.d/ros2.list && \
    apt-get remove -y \
    python3-catkin-pkg \
    python3-catkin-pkg-modules

# Install ROS 1 from Ubuntu packages.
RUN apt-get update && \
    apt-get install -y ros-core-dev

# Build ros1_bridge from source.
RUN git clone https://github.com/ros2/ros1_bridge src/ros1_bridge
RUN source install/local_setup.bash && \
    colcon build --packages-select ros1_bridge --cmake-force-configure

# Other configurations
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
RUN apt-get update && apt-get install -y ros-core-dev python3-packaging libspdlog-dev

# Copy files from host and build stage
COPY --from=build /ros2_humble/install /ros2_humble/install

# Other configurations
USER user
WORKDIR /
CMD ["./start-bridge.sh"]