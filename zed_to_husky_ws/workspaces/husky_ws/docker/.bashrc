# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Source workspace environment
# Note: If you have not built your workspace yet, the following command will fail
source $ROS2_WS/install/setup.bash
# Source Clearpath environment
# Note: The setup.bash file is created by the script "/usr/sbin/clearpath-robot-generate",
#       do not modify this file manually, as it will be overwritten. Try to modify the
#       "/etc/clearpath/robot.yaml" file instead, and then run the script again.
source /etc/clearpath/setup.bash