FROM introlab3it/rtabmap:jammy

LABEL org.opencontainers.image.authors="assume0701@gmail.com"
LABEL description = \
        "A image for ROS2 humble and RTAB-Map develop environment."

ARG USERNAME=user

#############################################
# --- Basic setup ---
RUN apt-get update && \
    apt-get install -y \
                sudo \
                vim \
                curl \
                tree \
                tmux \
                # ---
                # Simulation required packages
                ros-humble-turtlebot3 \
                ros-humble-turtlebot3-gazebo \
                ros-humble-gazebo-ros-pkgs \
                ros-humble-joint-state-publisher-gui \
                # ---
                # RTAB-MAP required packages
                ros-humble-rtabmap-ros && \
                rm -rf /var/lib/apt/lists/*

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

# add user with default bash
RUN adduser --disabled-password --gecos '' --shell /bin/bash ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Login with user
USER ${USERNAME}

CMD ["bash"]