# Cartographer

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_cartographer.sh)

ROS2 Cartographer. (Only installed by default for the template workspace.)

> Last tested on TODO.

To enable Cartographer, set the `CARTOGRAPHER` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `husky_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

> TODO: Remove duplicate content from the template workspace below.

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/template_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may replace the `docker compose build` command with `docker compose pull`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-template-ws bash
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
  docker attach ros2-template-ws
  cd /home/ros2-essentials/template_ws
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
  ros2 run rqt_robot_steering rqt_robot_steering
  ```

## Building Packages

```sh
cd /home/ros2-essentials/template_ws
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
  ros2 run rqt_robot_steering rqt_robot_steering
  ```

## References

- [Cartographer Demo](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)
