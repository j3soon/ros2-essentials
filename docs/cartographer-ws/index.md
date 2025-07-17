# Cartographer

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/cartographer_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-cartographer-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-cartographer-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=cartographer_ws)](https://github.com/j3soon/ros2-essentials/commits/main/cartographer_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--cartographer--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-cartographer-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64_|_arm64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-cartographer-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-cartographer-ws)

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

```sh
cd ~/ros2-essentials/cartographer_ws/docker
docker compose pull # or docker compose build
xhost +local:docker
docker compose up -d
```

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-cartographer-ws bash
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

## Simple Test With Turtlebot3

- Attach to the container

  ```sh
  docker attach ros2-cartographer-ws
  cd /home/ros2-essentials/cartographer_ws
  ```
- Open the turtlebot simulation in `tmux`

  ```bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
- Run the SLAM node in new window of `tmux`

  ```bash
  ros2 launch turtlebot3_cartographer cartographer.launch.py is_sim:=True
  ```
- Run the control tool in new window of `tmux`

  ```bash
  rqt_robot_steering
  ```

## Building Packages

```sh
cd /home/ros2-essentials/cartographer_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build
```

> After the build process, make sure to source the `install/setup.bash` file.  
> Otherwise, ROS2 will not locate the executable files. You can open a new terminal to accomplish this.

## Multi LiDAR - Single Robot SLAM test

### Simulation

- multi_lidar_desp package: Description of a robot with multiple LiDARs.
- multi_lidar_gazebo package: Gazebo simulation of the robot with robot state publisher.

  ```bash
  ros2 launch multi_lidar_gazebo multi_lidar_gazebo.launch.py
  ```

### Run the SLAM node

- Run the cartographer SLAM node in new window of `tmux`
  ```bash
  ros2 launch cartographer_demo cartographer_demo.launch.py is_sim:=True
  ```

- Run the control tool in new window of `tmux`
  ```bash
  rqt_robot_steering
  ```

## References

- [Cartographer Demo](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)
