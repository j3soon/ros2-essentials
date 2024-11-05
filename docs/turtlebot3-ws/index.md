# TurtleBot3

> Please note that this workspace is only tested in simulation.

## 🐳 Start Container

```sh
cd ~/ros2-essentials/turtlebot3_ws/docker
docker compose pull # or docker compose build
xhost +local:docker
docker compose up
# The initial build will take a while (~5 mins), please wait patiently.
```

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

(TODO)

## 🧩 Features

> Skip this section if you are not interested in enabling features.

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

## 💾 Save Output

> Skip this section if you are not interested in saving maps.

### Occupancy Grid Map

```sh
ros2 run nav2_map_server map_saver_cli -f $ROS2_WS/map
```

## 📌 References

- [TurtleBot3 \| ROBOTIS Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
