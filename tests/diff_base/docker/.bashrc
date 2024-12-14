{PLACEHOLDER_MULTILINE}
# Setup paths in `~/.profile` to allow unified environment variable across login/non-login shells
# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/bin" ] ; then
    PATH="$HOME/bin:$PATH"
fi
# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi
# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Optionally perform apt update if it has not been executed yet
if [ -z "$( ls -A '/var/lib/apt/lists' )" ]; then
    echo "apt-get update has not been executed yet. Running sudo apt-get update..."
    sudo apt-get update
fi
# Optionally perform rosdep update if it has not been executed yet
if [ ! -d $HOME/.ros/rosdep/sources.cache ]; then
    echo "rosdep update has not been executed yet. Running rosdep update..."
    rosdep update
    cd $ROS2_WS
    # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
    rosdep install --from-paths src --ignore-src -y -r
fi
# Optionally build the workspace if it has not been built yet
if [ ! -f $ROS2_WS/install/setup.bash ]; then
    echo "Workspace has not been built yet. Building workspace..."
    cd $ROS2_WS
    # TODO: If command `arch` outputs `aarch64`, consider adding `--packages-ignore <package>` to ignore x86 packages
    # Ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
    if [ $(arch) == "aarch64" ]; then
        colcon build --symlink-install{PLACEHOLDER}
    else
        colcon build --symlink-install
    fi
    echo "Workspace built."
fi
# Source gazebo environment
{PLACEHOLDER_MULTILINE}
# Ref: https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#InstallGazebo
if [ $(arch) == "x86_64" ]; then
  source /usr/share/gazebo/setup.bash
fi
# TODO: Source other workspace environments as underlay
{PLACEHOLDER_MULTILINE}
# Source workspace environment
source $ROS2_WS/install/setup.bash
echo "Successfully built workspace and configured environment variables."
