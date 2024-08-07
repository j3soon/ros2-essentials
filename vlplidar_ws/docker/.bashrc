# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Source workspace environment

# Check if the workspace has been built by check install/setup.bash
if [ ! -f $ROS2_WS/install/setup.bash ]; then
    echo "Workspace has not been built yet. Building workspace..."
    cd $ROS2_WS
    
    # If command "arch" is "aarch64", don't build velodyne_gazebo_plugins
    if [ $(arch) == "aarch64" ]; then
        colcon build --symlink-install --packages-ignore velodyne_gazebo_plugins
    else
        colcon build --symlink-install
    fi

    echo "Workspace built."
fi

# Note: If you have not built your workspace yet, the following command will fail
source $ROS2_WS/install/setup.bash
