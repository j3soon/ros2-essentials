# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Check the current folder is the workspace folder
# If it is, start building the workspace
if [ "$PWD" = "$ROS2_WS" ]; then
    echo "Found workspace, start building workspace"
    colcon build --symlink-install
fi

# Source workspace environment
# Note: If you have not built your workspace yet, the following command will fail
source $ROS2_WS/install/setup.bash