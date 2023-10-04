FROM osrf/ros:humble-desktop-full

LABEL org.opencontainers.image.authors="assume0701@gmail.com"
LABEL description = \
        "A image for ROS2 humble and ORB-SLAM3 develop environment."

ARG USERNAME=user

RUN apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

#############################################
# --- Basic setup ---
RUN apt-get update && \
    apt-get install -y \
                # Basic tools
                vim \
                curl \
                tree \
                wget \
                tmux \
                # ORB-SLAM dependencies
                libopencv-dev \
                opencv-data \
                libglew-dev \
                ffmpeg \
                libboost-system-dev \
                libeigen3-dev \
                # ROS dependencies
                ros-$ROS_DISTRO-vision-opencv \
                ros-$ROS_DISTRO-message-filters \
                # Python related
                python3-pip && \
                rm -rf /var/lib/apt/lists/*

# add user with default bash
RUN adduser --disabled-password --gecos '' --shell /bin/bash ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

#############################################
# --- Build Pangolin from source ---
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# get pangolin sources, certain commit : https://github.com/stevenlovegrove/Pangolin/tree/d484494645cb7361374ac0ef6b27e9ee6feffbd7
RUN mkdir -p /home/${USERNAME}/Pangolin/build && \
    cd /home/${USERNAME}/Pangolin && \
    git init && \
    git remote add origin https://github.com/stevenlovegrove/Pangolin.git && \
    git pull origin d484494645cb7361374ac0ef6b27e9ee6feffbd7 && \
    git rm -r --cached .

WORKDIR /home/${USERNAME}/Pangolin/build

# build pangolin
RUN cmake .. && \
    cmake --build .

#############################################
# --- Build ORB-SLAM3 from source
WORKDIR /home/${USERNAME}

RUN mkdir -p /home/${USERNAME}/ORB_SLAM3 && \
    cd /home/${USERNAME}/ORB_SLAM3 && \
    git init && \
    git remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git && \
    git pull origin 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4 && \
    git rm -r --cached .

# Change to c++14 for compile error
# Ref : https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/387
# Ref : https://github.com/Soldann/MORB_SLAM/issues/3
RUN sed -i 's/++11/++14/g' ORB_SLAM3/CMakeLists.txt && \
    sed -i 's/++11/++14/g' ORB_SLAM3/Thirdparty/g2o/CMakeLists.txt && \
    sed -i 's/++11/++14/g' ORB_SLAM3/Thirdparty/Sophus/CMakeLists.txt

RUN cd ORB_SLAM3 && \
    ./build.sh

RUN cd ORB_SLAM3/Thirdparty/Sophus/build && \
    sudo make install

#############################################
# --- Build ROS2-SLAM3
RUN mkdir -p /home/${USERNAME}/test_ws/src

WORKDIR /home/${USERNAME}/test_ws

RUN colcon build

RUN mkdir -p src/orbslam3_ros2 && \
    cd src/orbslam3_ros2 && \
    git init && \
    git remote add origin https://github.com/zang09/ORB_SLAM3_ROS2.git && \
    git pull origin 00c54335ccc010d74c1e24e336aa817604124947 && \
    git rm -r --cached .

# Fix path error and colcon build error
# Ref : https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970
RUN sed -i 's/foxy\/lib\/python3.8/humble\/lib\/python3.10/g' src/orbslam3_ros2/CMakeLists.txt && \
    sed -i 's/Pangolin)/Pangolin OpenCV)/g' src/orbslam3_ros2/CMakeLists.txt && \
    sed -i 's/Install\/ORB_SLAM\/ORB_SLAM3/ORB_SLAM3/g' src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake

RUN rosdep update && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select orbslam3

# Create work directory and went in
WORKDIR /home/${USERNAME}/ros_ws

CMD ["bash"]
