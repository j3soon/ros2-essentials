# Realsense

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_cartographer.sh)

ROS2 Realsense. (Only installed by default for the template workspace.)

To enable Realsense, set the `REALSENSE` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `husky_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

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

## Realsense test with gazebo

- realsense_desp package: Description of a robot with realsense D435 camera.
- realsense_gazebo package: Gazebo simulation of the robot with robot state publisher.
- Launch Gazebo with turtlebot3 in `tmux`
  ```bash
  ros2 launch realsense_gazebo realsense_gazebo.launch.py
  ```

- Run the control tool in new window of `tmux`
  ```bash
  ros2 run rqt_robot_steering rqt_robot_steering
  ```

## Realsense test in realworld

- Launch Realsense camera node in `tmux`
    ```bash
    # add pointcloud.enable:=true to enable pointcloud
    ros2 launch realsense2_camera rs_launch.py
    ```

- Open RViz to verify the camera topics in `tmux`
    ```bash
    ros2 run rviz2 rviz2
    ```

## References

- [Realsense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [RealSense Gazebo ROS Plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
