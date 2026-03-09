# ur_onrobot_rg2_ros
ROS packages for OnRobot RG2 gripper

# Examples

To run the ROS packages provided in this repository using a Docker configuration, please refer to the examples available in [ur-docker](https://github.com/husarion/ur-onrobot-rg2-docker) repo.

## ROS Nodes

### rg2_driver.py

OnRobot RG2 gripper MODBUS-based ROS controller.

#### Publishes

- `/joint_states` [*sensor_msgs/JointState*]: the current state of the gripper joint.

#### Services advertised

- `rg2/set_gripper_width` [*onrobot_rg2_driver/GripperState*]: Adjust the width of the RG2 gripper within a range of 0-100 mm. Gripper force can be set in the range of 3 to 40 [N].

#### Parameters

- `~ur_robot_ip` [*string*, default: **10.15.20.4**]: IP address of the UR robot.
- `~base_link_frame` [*string*, default: **rg2_base_link**]: name of the gripper base link frame.
- `~gripper_joint_name` [*string*, default: **rg2_gripper_joint**]: name of the gripper joint.
- `~service_call_timeout_sec` [*float*, default: **10.0**]: `/rg2/set_gripper_width` service timeout limit. 