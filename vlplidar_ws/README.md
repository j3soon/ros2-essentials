# Velodyne Workspace

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
roslaunch velodyne_pointcloud VLP16_points.launch
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
