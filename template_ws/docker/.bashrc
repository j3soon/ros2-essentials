# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Check if the workspace has been built by check install/setup.bash
if [ ! -f $ROS2_WS/install/setup.bash ]; then
    echo "Workspace has not been built yet. Building workspace..."
    cd $ROS2_WS
    # TODO: If command `arch` outputs `aarch64`, consider adding `--packages-ignore <package>` to ignore x86 packages
    if [ $(arch) == "aarch64" ]; then
        colcon build --symlink-install
    else
        colcon build --symlink-install
    fi
    echo "Workspace built."
fi
# Source workspace environment
source $ROS2_WS/install/setup.bash
