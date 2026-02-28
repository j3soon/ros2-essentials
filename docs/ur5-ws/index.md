# Universal Robots UR5

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/ur5_ws)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=ur5_ws)](https://github.com/j3soon/ros2-essentials/commits/main/ur5_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--ur5--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-ur5-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-ur5-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-ur5-ws)

This workspace provides a ROS 2 Humble environment for controlling **Universal Robots UR5** manipulators. It includes the official [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), description packages, **OnRobot RG2** gripper support, **servo control**, and **pose tracking**.

> Please note that this workspace has only been tested on the UR5 (CB3 version). In theory, it can support other models with minor configuration adjustments; please feel free to modify the settings as needed for your specific use case.

## Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/ur5_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may use `docker compose pull` instead of `docker compose build`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-ur5-ws bash
```

If the initial build failed, rebuild the workspace inside the container:

```sh
cd /home/ros2-essentials/ur5_ws
rm -rf build install
colcon build --symlink-install
```

To stop and remove the container:

```sh
docker compose down
```


## Simulation with URSim (CB3)

To test without hardware, run **URSim CB3** with the External Control URCap in Docker:

```sh
cd ~/ros2-essentials/ur5_ws/docker/ursim
docker compose up
```

This starts a UR5 simulator at `192.168.56.101` (VNC on port 5900, Web on 6080). The ports 5900 and 6080 are forwarded to localhost, please change them in `docker/ursim/compose.yaml` if those ports conflict with existing services. Open <http://localhost:6080/vnc.html?host=localhost&port=6080> in your browser to access the URSim interface.

> ⚠️ Important: If you plan to switch to the physical robot, you must run `docker compose down` to remove the virtual network first. Failing to do so will cause IP address conflicts.

URSim provides an interface identical to the physical robot, making it ideal for rapid testing. However, please note that unlike `Gazebo` or `IsaacSim`, URSim does not possess a realistic physics engine. Therefore, it is not suitable for simulations requiring high physical fidelity.

Regarding the RG2 gripper: although it is not explicitly modeled in URSim, our control script operates via Digital I/O. Consequently, there is no need to disable the gripper configuration when running URSim; the code will execute without errors.

Since URSim aims to faithfully replicate the real robot's behavior, you should follow all the steps outlined in the subsequent sections, including calibration and bring-up.

## Setup Robot

> For URSim users, the settings are already pre-configured in the docker compose file, so you can skip this step.

Please refer to the [official installation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html) guide. If the robot has been configured previously, you can typically skip this step.

For network setup, please refer to [this section](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/network_setup.html) for network configuration.

## Robot Calibration

To improve accuracy, run the calibration correction and save the result to the workspace description config:

```sh
cd /home/ros2-essentials/ur5_ws
./scripts/ur5_calibration.sh
```

You only need to run this once for each robot. This creates `default_kinematics.yaml` which is required for the subsequent steps.

## Bring Up Robot

Use the script:

```sh
cd /home/ros2-essentials/ur5_ws
./scripts/ur5_bringup.sh
```

Or run the launch file directly:

```sh
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 \
    use_rg2_gripper:=true \
    robot_ip:=192.168.56.101 \
    launch_rviz:=true
```

> Set `use_rg2_gripper` parameter to **false** to disable the RG2 gripper.

Quick test for gripper actuation:

```sh
# true = open, false = close
ros2 topic pub --once /rg2/command std_msgs/msg/Bool "{data: true}"
```

After bringing up the robot, make sure to click `Play` in the UR interface to start the control loop.

In the UR interface, go to `Home > Program Robot > Empty Program > Program > Structure > URCaps > External Control` and then click the Play button.

![](assets/ursim-external-control-running.png)

The UR interface control program will automatically stop if the ROS2 bring up node is stopped or crashes for safety reasons. If that happens, simply restart the bring up node and click `Play` again.

## Launch MoveIt 2

Launch MoveIt 2 for a quick test. You can move the arm by dragging the end-effector (EE) to a target position in RViz2, then clicking `Plan & Execute`.

```sh
ros2 launch ur_moveit_config ur_moveit.launch.py \
    launch_rviz:=true \
    use_rg2_gripper:=true
```

> Ensure the robot driver is already running before testing.

## Servo Control

The **ur_servo_control** package provides a simple example for controlling end-effector (EE) velocity. Please ensure the robot driver is launched before testing.

```sh
ros2 launch ur_servo_control ur_servo.launch.py
```

You can control the robot arm by publishing target velocities to the `/servo_node/delta_twist_cmds` topic.

For an implementation example within a ros node, refer to `ur5_ws/src/ur_servo_control/ur_servo_control/ur_servo_control.py`. Please be careful when selecting the reference frame. Use the following command to launch it:

```sh
ros2 launch ur_servo_control ur_servo_control.launch.py
```

The default velocities above are set to 0, you can modify the linear/angular velocities (`twist_msg.twist.{linear|angular}.{x|y|z}`) in `publish_servo_command_callback` in `ur5_ws/src/ur_servo_control/ur_servo_control/ur_servo_control.py`. But note that a constant velocity command will cause the robot to keep moving, which may be dangerous for real robot control (but fine for URSim testing).

## Pose Tracking

The **ur_pose_tracking** package provides an interface for controlling the end-effector (EE) position.

```sh
# In terminal 1 (Launch the servo node):
ros2 launch ur_servo_control ur_servo.launch.py
# In terminal 2:
ros2 launch ur_pose_tracking ur_pose_tracking.launch.py
```

You can control the robot arm by publishing target pose to the `/target_pose` topic.

For application integration, it is recommended to refer to the **servo control implementation within the ros node**. To perform a quick test, please use the following command:

```sh
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world', stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: -0.3, y: 0.0, z: 0.5}, orientation: {x: -0.35, y: 0.01, z: 0.93, w: -0.05}}}"
```

> Note: This implementation does not account for singularities or collision avoidance and only supports linear paths between points. Consequently, it is ideal for high-density trajectory points. If your input points are sparse or require complex path planning and obstacle avoidance, please use the MoveIt planner instead.

## Troubleshooting

If you encounter issues on reproducing the examples above, consider remove all containers (`docker compose down` for URSim and workspace) and try again.

## References

- [Universal Robots ROS 2 Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)
  - [Robot Setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html)
  - [Setup URSim with Docker](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html)
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
