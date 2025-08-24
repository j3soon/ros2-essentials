FROM osrf/ros:humble-desktop-full

LABEL org.opencontainers.image.authors="assume0701@gmail.com"
LABEL description = \
        "An image for ROS2 VLP LiDAR."

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
                ros-humble-rqt-robot-steering \
                ros-humble-diagnostic-updater \
                ros-humble-velodyne && \
                rm -rf /var/lib/apt/lists/*

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

# # add user with default bash
RUN adduser --disabled-password --gecos '' --shell /bin/bash ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# COPY .bashrc /home/$USERNAME/.bashrc

# Login with user
USER ${USERNAME}

CMD ["bash"]