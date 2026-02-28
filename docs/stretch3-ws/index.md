# Hello Robot Stretch 3

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/stretch3_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-stretch3-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-stretch3-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=stretch3_ws)](https://github.com/j3soon/ros2-essentials/commits/main/stretch3_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--stretch3--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-stretch3-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-stretch3-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-stretch3-ws)

> Please note that this workspace is only tested in simulation.

> Last tested on commit [a9e1166](https://github.com/j3soon/ros2-essentials/commit/a9e116664353b8d874d7e559d023c625b08c9dd1) by [@j3soon](https://github.com/j3soon).

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/stretch3_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may replace the `docker compose build` command with `docker compose pull`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-stretch3-ws bash
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

> If you encountered unexpected issues when using Isaac Sim, see [Known Issues](../docker-modules/isaac-sim.md#known-issues) for more details.

### Isaac Sim Keyboard Control

Run `~/isaacsim/isaac-sim.sh` and open `/home/ros2-essentials/stretch3_ws/isaacsim/assets/stretch3_og_wasd.usda` in Isaac Sim, and then press Play and use WASD to control the robot.

<video controls>
  <source src="assets/stretch3_og_wasd.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

### ROS2 Keyboard Control

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### RTAB-Map SLAM Demo

Use tmux to open multiple terminals:

1. Run `~/isaacsim/isaac-sim.sh` and open `/home/ros2-essentials/stretch3_ws/isaacsim/assets/stretch3_og_wasd.usda` in Isaac Sim, and then press Play.

2. Launch rtabmap SLAM:
```sh
ros2 launch stretch3_navigation rtabmap.launch.py
```

3. Launch rviz2:
```sh
ros2 launch stretch3_navigation rviz.launch.py
```

In rviz2, use `2D Goal Pose` to set a goal pose for the robot. Nav2 system will guide the robot to the goal pose.

<img src="assets/stretch3_rtabmap.png" alt="RTAB-Map SLAM Demo" width="800"/>

### Cartographer SLAM Demo

Use tmux to open multiple terminals:

1. Run `~/isaacsim/isaac-sim.sh` and open `/home/ros2-essentials/stretch3_ws/isaacsim/assets/stretch3_og_wasd.usda` in Isaac Sim, and then press Play.

2. Launch cartographer SLAM:
```sh
ros2 launch stretch3_navigation cartographer.launch.py
```

3. Launch rviz2:
```sh
ros2 launch stretch3_navigation rviz.launch.py
```

In rviz2, use `2D Goal Pose` to set a goal pose for the robot. Nav2 system will guide the robot to the goal pose.

<img src="assets/stretch3_cartographer.png" alt="Cartographer SLAM Demo" width="800"/>  

### Known Issues

If the robot moves too fast, it will cause skidding and the robot will need sometime to correct itself.

Also, the robot turning very slowly, sometimes it stuck when turning around.

### References

- [Stretch Docs](https://docs.hello-robot.com/0.3/)
  - [Stretch 3 Hardware Guide - Stretch Docs](https://docs.hello-robot.com/0.3/hardware/hardware_guide_stretch_3)
- [Hello Robot GitHub](https://github.com/hello-robot)
  - [hello-robot/stretch_urdf](https://github.com/hello-robot/stretch_urdf)
