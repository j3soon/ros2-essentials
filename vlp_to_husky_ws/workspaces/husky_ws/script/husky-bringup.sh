#!/bin/bash -e

# This script is used to bring up the Husky robot in real world.
# It is assumed that the robot is powered on and the computer is connected to the robot via USB cable.

# Note: As the script will initiate ROS nodes in the background, ending the script won't halt the ROS nodes. 
#       Typically, you'd use the stop script provided by Clearpath located at `/usr/sbin/clearpath-platform-stop`. 
#       However, this stop script won't function as expected, it solely terminates this script, not the ROS nodes. 
#       You can attempt to manually terminate the ROS nodes or exit the terminal session.

/usr/sbin/clearpath-platform-start

# Originally, the start script is executed by the service, which is configured during the installation step.
# However, within the Docker container, we cannot enable the service, so we must run the start script manually.
# Reference:
# - https://github.com/clearpathrobotics/clearpath_computer_installer/blob/7e7f41599ea851d3a618f00afe4f8f35c91eac53/clearpath_computer_installer.sh#L294-L310
# - https://github.com/j3soon/docker-ros-husky/blob/1b1ba2683153b909405dd93b0d2b8ac84f555641/docker-exec-bringup.sh