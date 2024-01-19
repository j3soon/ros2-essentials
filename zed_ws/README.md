# zed_ws

# ◻️ Introduction ◻️

This workspace is used to configure a ROS2 sensor for ZED that can seamlessly switch between real and simulation modes.

In simulation mode, we won't be utilizing the entire ZED SDK. Consequently, features such as `Object Detection` or `Body Tracking` will not be available. The Gazebo plugin  only focuses on publishing essential data, such as the `camera raw image`. Additional data, like the `Gray Image`, requires a separate node for publication, which hasn't been implemented in this workspace.

# 🚩 How to use 🚩

> For convenience, let's assume you are using the `ZED 2i` camera.

Most commands in the [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and [zed-ros2-example](https://github.com/stereolabs/zed-ros2-examples/tree/master) are functional, except for the simulation feature in Omniverse, which is currently disabled. We aim to simulate the camera in Gazebo within this workspace.

Below are some crucial commands you should be familiar with.

## Build the workspace

```bash=
cd /home/ros2-agv-essentials/zed_ws
colcon build --symlink-install
```

## Starting the ZED node

```bash=
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

> For simulation mode, append the flag `sim_mode:=true` at the end.

## Rviz visualization

```bash=
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```

> For simulation mode, append the flag `sim_mode:=true` at the end.

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
