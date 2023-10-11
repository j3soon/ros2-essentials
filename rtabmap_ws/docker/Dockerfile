FROM introlab3it/rtabmap:jammy

ARG USERNAME=user

#############################################
# --- Basic setup ---
RUN apt-get update && \
    apt-get install -y \
                sudo \
                vim \
                curl \
                tree \
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
WORKDIR /home/${USERNAME}

# Create work directory and went in
WORKDIR /home/${USERNAME}/ros_ws

CMD ["bash"]