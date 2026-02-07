# TurtleBot3

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/turtlebot3_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-turtlebot3-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-turtlebot3-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=turtlebot3_ws)](https://github.com/j3soon/ros2-essentials/commits/main/turtlebot3_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--turtlebot3--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-turtlebot3-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-turtlebot3-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-turtlebot3-ws)

> Please note that this workspace is only tested in simulation.

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/turtlebot3_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may replace the `docker compose build` command with `docker compose pull`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-turtlebot3-ws bash
```

TurtleBot3 `burger` is used by default, you can change it by the following (such as `waffle` or `waffle_pi`):

```sh
export TURTLEBOT3_MODEL=waffle_pi
```

If the initial build somehow failed, run:

```sh
rm -r build install
colcon build --symlink-install
```

Once you have finished testing, you can stop and remove the container with:

```sh
docker compose down
```

## 🔍 Inspect Robot Model

> Skip this section if you are not interested in inspecting the URDF/USD model.

(TODO)

## 🪐 Launch Simulator

Launch one of the following simulators (1) RViz Fake Node (2) Gazebo (3) Isaac Sim.

### RViz Fake Node

Note that RViz fake node doesn't simulate sensors such as LiDAR.

```sh
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
```

### Gazebo

Launch one of the following worlds:

```sh
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### Isaac Sim

Convert URDF file to USD file and generate OmniGraph:

```sh
cd /home/ros2-essentials/turtlebot3_ws/isaacsim/scripts
./create_urdf_from_xacro.sh
~/isaacsim/python.sh create_turtlebot3_burger_from_urdf.py
~/isaacsim/python.sh create_turtlebot3_burger_with_omnigraph.py
# and close the window when you see the `Done` message (or seeing segmentation fault)
```

> Note that it is entirely unnecessary to convert URDF to USD with scripts, this is just for demonstration purposes. You can directly use the GUI to import the URDF file, and track it in Git easily with USDA. See [stretch3-ws](../stretch3-ws/) for more details.
>
> It is also unnecessary to generate OmniGraph with scripts, you can directly use the GUI to create it, and track it in Git easily with USDA. See [stretch3-ws](../stretch3-ws/) commit history for more details.

Start Isaac Sim in GUI mode:

```sh
~/isaacsim/isaac-sim.sh
```

> Alternatively, start Isaac Sim in [headless WebRTC mode](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_python.html#launching-isaac-sim-experiences):
> 
> ```sh
> isaac-sim.streaming.sh
> ```
>
> and use the [WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/manual_livestream_clients.html#isaac-sim-short-webrtc-streaming-client).

Open the file with OmniGraph we just generated in the bottom panel:

```
/home/ros2-essentials/turtlebot3_ws/isaacsim/assets/turtlebot3_burger_og.usd
```

or use `File > Open Recent` to re-open it, and click `Play (SPACE)` to start simulation.

> There is currently a known issue on the 2D Lidar sensor and are waiting for an [upstream fix](https://github.com/isaac-sim/IsaacSim/issues/438).
>
> ```
> [Error] [isaacsim.sensors.rtx.plugin] IsaacComputeRTXLidarFlatScan: Lidar prim is not a 2D Lidar, and node will not execute. Stop the simulation, correct the issue, and restart.
> ```
>
> (TODO: Fix `turtlebot3_burger_isaacsim.urdf.xacro` and remove this block after the upstream fix is released.)

To view the OmniGraph, right click the ActionGraph on the right panel and select `Open Graph`.

## 🧩 Features

> Skip this section if you are not interested in enabling features.

> Note that Cartographer and Nav2 features are only supported in Gazebo simulation for now.

### Cartographer

Basic LiDAR SLAM.

```sh
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Nav2

Requires a running SLAM or a pre-generated map.

```sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$ROS2_WS/map.yaml
```

## 🕹️ Control Robot

### Raw Message

Move forward:

```sh
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.2, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```

Stop:

```sh
ros2 topic pub /cmd_vel geometry_msgs/Twist "{'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}"
```

### Teleoperate

Teleoperate through keyboard:

```sh
ros2 run turtlebot3_teleop teleop_keyboard
```

### Simple Auto Drive

Simple collision avoidance:

```sh
ros2 run turtlebot3_gazebo turtlebot3_drive
```

### Visualize with RViz

Open a new terminal and visualize published topics in RViz:

```sh
ros2 launch turtlebot3_bringup rviz2.launch.py
```

Set RViz GUI:

- `Fixed Frame`: `world`.
- Add `Camera` with topic `/rgb`.
- (TODO: Add LaserScan, PointCloud, and Depth after the upstream fix is released.)

## 💾 Save Output

> Skip this section if you are not interested in saving maps.

### Occupancy Grid Map

```sh
ros2 run nav2_map_server map_saver_cli -f $ROS2_WS/map
```

## 📌 References

- [TurtleBot3 \| ROBOTIS Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
