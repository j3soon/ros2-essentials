#!/bin/bash

if [ $# = 1 ]; then
    export ROS_MASTER_URI=$1
else
    export ROS_MASTER_URI=http://localhost:11311
fi

source /ros2_humble/install/setup.bash
source /ros2_humble/install/ros1_bridge/share/ros1_bridge/local_setup.bash

ros2 run ros1_bridge dynamic_bridge