# Base Image : https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:humble-desktop-full

LABEL org.opencontainers.image.authors="assume0701@gmail.com"
LABEL description = \
        "A image for ROS2 humble and slam-toolbox develop environment."

ARG USERNAME=user

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
                vim \
                curl \
                tree \
                tmux \
                git \
                htop \
                net-tools \
                # --- 
                # Simulation required packages
                ros-humble-turtlebot3 \
                ros-humble-turtlebot3-gazebo \
                ros-humble-gazebo-ros-pkgs \
                ros-humble-joint-state-publisher-gui \
                # ---
                # SLAM-TOOLBOX required packages 
                ros-humble-slam-toolbox && \
                # ---
                # Clean up
                rm -rf /var/lib/apt/lists/*

# Add user with default bash
RUN adduser --disabled-password --gecos '' --shell /bin/bash ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

COPY .bashrc /home/$USERNAME/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}

CMD ["bash"]
