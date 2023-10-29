#!/bin/bash -e

if [ $# = 1 ]; then
    export ROS_MASTER_URI=$1
else
    export ROS_MASTER_URI=http://localhost:11311
fi

# Normally, sourcing the setup.bash file is sufficient to set up the ROS environment.
# However, the ros1_bridge package can't be found by ROS 2 even if we source the file. Manual sourcing is required.
source /ros2_humble/install/setup.bash
source /ros2_humble/install/ros1_bridge/share/ros1_bridge/local_setup.bash

ros2 run ros1_bridge dynamic_bridge