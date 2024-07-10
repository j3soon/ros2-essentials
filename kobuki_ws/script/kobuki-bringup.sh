#!/bin/bash -e

# Running udevd in the background, it will listen for events from the kernel and manage the device nodes in /dev.
# Reference: 
# - https://forums.docker.com/t/udevadm-control-reload-rules/135564/2
# - https://manpages.ubuntu.com/manpages/focal/en/man8/systemd-udevd-kernel.socket.8.html
if ! pidof "systemd-udevd" > /dev/null; then
    echo "Launching systemd-udevd ..."
    sudo /lib/systemd/systemd-udevd --daemon &> /dev/null
fi

echo "Reloading the udev rules ..."

# Reload the udev rules.
# Reference: https://unix.stackexchange.com/a/39371
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

echo "Done."

# Source kobuki driver workspace environment.
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/kobuki_driver_ws/install/local_setup.bash

# Bring up the kobuki node.
ros2 launch kobuki_node kobuki_node-launch.py