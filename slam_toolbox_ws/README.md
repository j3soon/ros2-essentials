# ROS2 SLAM-TOOLBOX

### Run with docker

```bash
git clone https://github.com/j3soon/ros2-agv-essentials.git
```

```bash
cd ros2-agv-essentials/slam_toolbox_ws/docker
docker-compose pull
docker-compose up -d --build
```

### Simulation with gazebo

- Attach to the container
  ```sh
  docker attach ros2-slam-toolbox-ws
  ```

- Run with the gazebo
```sh
ros2 launch slam_simulation slam_sim.launch.py
```

You can put something in the gazebo world, since currently is empty.

#### Reference repo

- [slam-toolbox humble](https://github.com/SteveMacenski/slam_toolbox/tree/humble)
- [Nav2 slam_toolbox guide](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
- [Robotics Backend Guide for slamtool box and nav2](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/)