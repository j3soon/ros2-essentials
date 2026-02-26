# Realsense

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_realsense.sh)

ROS2 Realsense. (Only installed by default for the template workspace.)

To enable Realsense, set the `REALSENSE` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `husky_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

> **Note:** These scripts have only been tested on the **RealSense D435i**.

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

## Simulation

### Realsense test with gazebo

- realsense_gazebo package: Gazebo simulation of the robot with RealSense D435 camera.
- Launch Gazebo with turtlebot3 in `tmux`
  ```bash
  ros2 launch realsense_gazebo realsense_bringup.launch.py
  ```

- Run the control tool in new window of `tmux`
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

## Real Hardware

### Realsense test in real world

- Launch Realsense camera node in `tmux`
    ```bash
    # add pointcloud.enable:=true to enable pointcloud
    ros2 launch realsense2_camera rs_launch.py
    ```

- Open RViz to verify the camera topics in `tmux`
    ```bash
    rviz2
    ```

### Tools

Realsense viewer:

```sh
realsense-viewer
```

### Updating Firmware

Updating firmware via `realsense-viewer` [will fail at 30%](https://github.com/realsenseai/librealsense/issues/14345) if `systemd-udevd` is not running, which could result in the following error:

```
Recovery device did not connect in time!
```

Make sure to start `systemd-udevd` in the container before updating firmware:

```sh
sudo /lib/systemd/systemd-udevd --daemon
realsense-viewer
# and then update firmware via realsense-viewer
```

`systemd-udevd` is required to detect when the (recovery) device reconnects after requesting the camera to switch to recovery mode.

> Example output of a successful firmware update:
>
> ```
> Started Firmware Update process
> Trying to back-up camera flash memory
> Backup completed and saved as '/home/user/.xxxxxxxxxxxx.xxxxxxxx_xxxxxx.bin'
> Requesting to switch to recovery mode
> DFU device 'xxxxxxxxxxxx' found
> Recovery device connected, starting update..
> Internal write is in progress
> Please DO NOT DISCONNECT the camera
> Firmware Download completed, await DFU transition event
> Discovered connection of the original device
> Device reconnected successfully!
> FW update process completed successfully
> ```

### Troubleshooting

Check USB devices (on host):

```sh
lsusb
```

Check udev rules:

```sh
ls -l /lib/udev/rules.d/ | grep realsense
```

Show extrinsic and intrinsics:

```sh
rs-enumerate-devices -c
```

Show serial number:

```sh
rs-enumerate-devices -s
```

List firmware version:

```sh
rs-fw-update -l
```

## References

- [Realsense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [RealSense Gazebo ROS Plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [Troubleshooting Q&A](https://github.com/realsenseai/librealsense/wiki/Troubleshooting-Q%26A)
- [Troubleshooting Tips and Tricks](https://dev.realsenseai.com/docs/troubleshooting)
- [Librealsense Docker](https://github.com/realsenseai/librealsense/tree/master/scripts/Docker)
