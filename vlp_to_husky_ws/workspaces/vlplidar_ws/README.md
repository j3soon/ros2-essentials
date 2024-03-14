# Velodyne Workspace

## Simulation Setup

### Add description in defined robot

1. Declared necessary argument and  your own `robot_gazebo.urdf.xacro`

    ```xml
    <xacro:arg name="gpu" default="false"/>
    <xacro:arg name="organize_cloud" default="false"/>
    <xacro:property name="gpu" value="$(arg gpu)" />
    <xacro:property name="organize_cloud" value="$(arg organize_cloud)" />
    ```

2. Include the `velodyne_description` package

    ```xml
    <xacro:include filename="$(find velodyne_description)/urdf/VLP16.urdf.xacro" />
    ```

3. Add LiDAR in the robot

    ```xml
    <xacro:VLP-16 parent="base_footprint" name="velodyne" topic="/velodyne_points" organize_cloud="${organize_cloud}" hz="10" samples="440" gpu="${gpu}">
        <origin xyz="0 0 0.1" rpy="0 0 0" />
    </xacro:VLP-16>
    ```

- You could refer to more information from `veloyne_simulator/velodyne_description/urdf/template.urdf.xacro`

---

## Launch LiDAR driver with simulated LiDAR

- Launch example robot with LiDAR

    ```bash
    ros2 launch velodyne_simulator example.launch.py
    ```

- Launch the LiDAR data process without driver in another terminal

    ```bash
    ros2 launch vlp_cartographer vlp_driver.launch.py is_sim:=True
    ```

### Sample Robot

- A given turtlebot waffle model with VLP-16 is provided in `velodyne_simulator/velodyne_simulator/urdf/sample_robot.urdf.xacro`
- Launch the sample robot

    ```bash
    ros2 launch velodyne_description example.launch.py robot:=1
    ```

---

## LiDAR setup

### Hardware Setup

- Connect the LiDAR to power.
- Connect the LiDAR to the computer or router using the provided ethernet cable.

#### Directly Using Computer

- Connect the LiDAR to the computer using the ethernet cable.
- Open the computer settings and navigate to **Network** > **Wired**.

<center>
    <img width="65%" src="assets/wireless-settings.png">
</center>

- Set the IPv4 configuration to 'manual' and configure the settings as shown in the image below:

<center>
    <img width="65%" src="assets/network-settings.png">
</center>

### Launch the Driver

#### Pipeline

- Data process as following:
  raw data -> pointcloud -> laser scan -> slam method

- Velodyne driver: `velodyne_driver` get the raw data from LiDAR.
- Transform the raw data to pointcloud: `velodyne_pointcloud`
- Transform the pointcloud to laser scan: `velodyne_laserscan`

#### Operating in a single launch file

```bash
ros2 launch vlp_cartographer vlp_driver.launch.py
```

- By the above command, the driver, pointcloud and laserscan will be launched.

#### Published topics

| Topic | Type | Description |
| -------- | -------- | -------- |
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | raw data |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Point cloud message |
| `/scan` | `sensor_msgs/LaserScan` | laser scan message |


---

## Test with cartographer

- In another terminal, launch the cartographer node:

    ```bash
    ros2 launch vlp_cartographer cartographer_demo.launch.py
    ```

---

## Reference

- [Velodyne_driver with ROS2 Humble](https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3)
- [Cartographer ROS](https://github.com/cartographer-project/cartographer_ros/tree/c138034db0c47fe0ea5a2abe516acae02190dbf5)
- [Turtlebot3 with Cartographer](https://github.com/ROBOTIS-GIT/turtlebot3/tree/a7dd05ae176f3f3778b0a36f7065dc9655b050e3)
