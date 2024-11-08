# zed_ws

# ◻️ Introduction ◻️

This workspace is used to configure a ROS2 sensor for ZED that can seamlessly switch between real and simulation modes.

In simulation mode, there are two platforms to choose from.

- For Gazebo, we won't be utilizing the entire ZED SDK. Consequently, features such as `Object Detection` or `Body Tracking` will not be available. The Gazebo plugin focuses solely on publishing essential data, such as the `camera raw image`. Additional data, like the `Gray Image`, requires a separate node for publication, which hasn't been implemented in this workspace.
- For Isaac Sim, it will utilize the ZED SDK. Please refer to the [official documents](https://www.stereolabs.com/docs/isaac-sim/isaac_sim) for more details. As we have provided the Dockerfile for Isaac Sim, there is no need to install Omniverse Launcher locally.

# 🚩 How to use 🚩

> For convenience, let's assume you are using the `ZED 2i` camera.

Most commands in the [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and [zed-ros2-example](https://github.com/stereolabs/zed-ros2-examples/tree/master) are functional. Below are some crucial commands you should be familiar with.

## Build the workspace

```bash=
cd /home/ros2-agv-essentials/zed_ws
colcon build --symlink-install
```

## Starting the ZED node

```bash=
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

## Rviz visualization

```bash=
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```

## Simulation

Append the flag `sim_mode:=true` at the end to enable the simulation mode. Use the flag `sim_platform` to specify the platform you want to simulate. The choices are `isaac_sim` and `gazebo`.

```bash=
# Simulation with Gazebo
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i sim_mode:=true use_sim_time:=true sim_platform:=gazebo

# Simulation with Isaac Sim
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx sim_mode:=true use_sim_time:=true sim_platform:=isaac_sim
```

> If you opt to simulate with Isaac Sim, you can utilize the provided Docker container. To launch Isaac Sim within the container, please follow the commands below:
> ```bash=
> # Outside the container
> cd ~/ros2-agv-essentials/zed_ws/docker
> docker compose build zed-isaac-sim
> docker compose run zed-isaac-sim
> 
> # Inside the container
> ./setup-isaac-sim.sh
> ./runapp.sh # ./runheadless.native.sh for headless mode
> ```
> 
> Initializing Isaac Sim may consume a significant amount of time. Please wait for notifications, such as "Isaac Sim App is loaded," in the terminal.  
> 
> Once all plugins have started up without errors in the terminal, you can proceed by following the [guide](https://www.stereolabs.com/docs/isaac-sim/ros2_integration#enable-the-ros-2-bridge-extension) provided by StereoLabs or opening the file we supplied. It is located at `/home/ros2-agv-essentials/zed_ws/src/zed-isaac-sim/usd/zed_tutorial.usd`. (Don't forget to install the ZED extension)

## Integration on a robot

We maintain the original structure of the URDF, preserving the same tf relations as the default setup. You can refer to the [official documentation](https://www.stereolabs.com/docs/ros2/ros2-robot-integration) for integration details directly. 

If you intend to simulate in Gazebo, make sure to include the following tags in your URDF file.

```xml=
<xacro:include filename="$(find zed_wrapper)/urdf/zed_gazebo.urdf.xacro" />
<xacro:zed_gazebo name="$(arg camera_name)" />
```

> Don't forget to remove the static tf `map -> zed_camera_link` in the file  
> `zed-ros2-wrapper/zed_wrapper/launch/zed_camera.launch.py`.

# 🔍 Troubleshooting 🔍

## Can't open the camera

```bash=
# Error message
[zed.zed_node]: Error opening camera: CAMERA NOT DETECTED
[zed.zed_node]: Please verify the camera connection
```

Before proceeding, confirm the connection to the ZED camera by using the `lsusb` command. If the camera is connected correctly, the information should be displayed, such as `Bus ... Device ...: ID 2b03:f881 STEREOLABS ZED-2i HID INTERFACE`.

If the issue persists even after confirming the connection, troubleshoot by checking your CUDA installation or attempting to open the camera again.

## MOTION SENSORS REQUIRED

```bash=
# WARN message
[zed.zed_node]: sl::getSensorsData error: MOTION SENSORS REQUIRED
```

> [From R&D - Stereolabs:](https://community.stereolabs.com/t/motion-sensors-required/2336)
> 
> The message `MOTION SENSORS REQUIRED` can reveal that your USB connection has issue. The ZED camera uses both USB2 and USB3. Maybe you have a faulty USB cable or port.
