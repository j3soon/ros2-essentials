FROM osrf/ros:humble-desktop

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
# --- Install required packages ---
RUN sudo apt install libopencv-dev opencv-data \
             libglew-dev ffmpeg libboost-system-dev libeigen3-dev -y


#############################################
# --- Build Pangolin from source ---
WORKDIR /home/${USERNAME}

# get pangolin sources
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
RUN mkdir -p /home/${USERNAME}/Pangolin/build
WORKDIR /home/${USERNAME}/Pangolin/build

# build pangolin
RUN cmake ..
RUN cmake --build .


#############################################
# --- Build ORB-SLAM3 from source
WORKDIR /home/${USERNAME}

RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ~/ORB_SLAM3

# Change to c++14 for compile error
# Ref : https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/387
# Ref : https://github.com/Soldann/MORB_SLAM/issues/3
RUN sed -i 's/++11/++14/g' ORB_SLAM3/CMakeLists.txt
RUN sed -i 's/++11/++14/g' ORB_SLAM3/Thirdparty/g2o/CMakeLists.txt
RUN sed -i 's/++11/++14/g' ORB_SLAM3/Thirdparty/Sophus/CMakeLists.txt

RUN cd ORB_SLAM3 && ./build.sh

RUN cd ORB_SLAM3/Thirdparty/Sophus/build && sudo make install

# build ros-orbslam
WORKDIR /home/${USERNAME}

RUN mkdir -p test_ws/src

WORKDIR /home/${USERNAME}/test_ws

RUN colcon build && git clone https://github.com/zang09/ORB_SLAM3_ROS2.git ./src/orbslam3_ros2

# Fix path error and colcon build error
# Ref : https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970
RUN sed -i 's/foxy\/lib\/python3.8/humble\/lib\/python3.10/g' src/orbslam3_ros2/CMakeLists.txt
RUN sed -i 's/Pangolin)/Pangolin OpenCV)/g' src/orbslam3_ros2/CMakeLists.txt
RUN sed -i 's/Install\/ORB_SLAM\/ORB_SLAM3/ORB_SLAM3/g' src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake

RUN sudo apt update && sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters -y

RUN rosdep update

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --packages-select orbslam3

# -----------------------------

# Create work directory and went in
WORKDIR /home/${USERNAME}/ros_ws

CMD ["bash"]