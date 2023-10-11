FROM introlab3it/rtabmap:jammy

ARG USERNAME=assume

#############################################
# --- Basic setup ---
RUN apt update && \
    apt install sudo vim curl tree -y

# Set the timezone
RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

# add user with default bash
RUN adduser --disabled-password --gecos '' --shell /bin/bash ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Login with user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

#############################################
# --- Install required ROS2 packages ---
RUN curl -sSL http://get.gazebosim.org | sh
RUN sudo apt update
RUN sudo apt install \
             # --- 
             # Simulation required packages
             ros-humble-turtlebot3 \
             ros-humble-turtlebot3-gazebo \
             ros-humble-gazebo-ros-pkgs \
             ros-humble-joint-state-publisher-gui \
             # ---
             # RTAB-MAP required packages 
             ros-humble-rtabmap-ros -y

# Create work directory and went in
WORKDIR /home/${USERNAME}/ros_ws

CMD ["bash"]