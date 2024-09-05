# interbotix_xs_ros_control

## Overview
This package provides the necessary ros2_control hardware interfaces needed to get MoveIt to control any physical Interbotix X-Series arm. It essentially takes in Joint Trajectory commands from MoveIt (via the FollowJointTrajectoryAction interface) and then publishes joint commands at the right time to the **xs_sdk** node. Currently, only the 'position' values in the Joint Trajectory messages are used since that provides the smoothest motion. Note that while this package is really only meant to be used with MoveIt, it could technically be used with any other node that can interface properly with the [joint_trajectory_controller](http://wiki.ros.org/joint_trajectory_controller) package.

## Nodes
The *interbotix_xs_ros_control* nodes are described below:
- **controller_manager** - responsible for loading and starting a set of controllers at once, as well as automatically stopping and unloading those same controllers
- **xs_hardware_interface** - receives joint commands from the ROS controllers and publishes them to the correct topics (subscribed to by the **xs_sdk** node) at the appropriate times

## Usage
This package is not meant to be used by itself but with any X-Series robot platform that contains an arm (like a standalone arm or a mobile manipulator). Refer to the example ROS packages by those robot platforms to see more info on how this package is used. These nodes are not located there to avoid code duplicity.
