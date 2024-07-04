#!/bin/bash -e

# This script is used to generate the configuration files for the Husky robot.
# It will use the file "/etc/clearpath/robot.yaml" to generate the configuration files.

# Replace the domain_id in the robot.yaml file with the value of ROS_DOMAIN_ID environment variable.
# This is necessary to ensure that the configuration files are generated with the correct domain id.
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo "Setting domain_id in /etc/clearpath/robot.yaml to $ROS_DOMAIN_ID."
    sed -i "s/domain_id: [0-9]\+/domain_id: $ROS_DOMAIN_ID/" /etc/clearpath/robot.yaml
fi

# Run the script.
/usr/sbin/clearpath-robot-generate