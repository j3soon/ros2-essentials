# ROS2 RTABMAP Docker

### Run with dockerfile

```bash
./run.sh
```

### Simple test

- Test in Gazebo with turtlebot3
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
- rtabmap demos ( Use LiDAR )
```bash
ros2 launch rtabmap_demos turtlebot3_scan.launch.py
```

### Reference

- [RTAB-Map wiki](https://github.com/introlab/rtabmap/wiki)