# ROS2 Cartographer

### Run with docker

```bash
git clone https://github.com/j3soon/ros2-agv-essentials.git
```

```bash
cd ros2-agv-essentials/cartographer_ws/docker
docker-compose pull
docker-compose up -d --build
```

### Simple Test With Turtlebot3

- Attach to the container

  ```sh
  docker attach ros2-cartographer-ws
  cd /home/ros2-agv-essentials/cartographer_ws
  ```
- Open the turtlebot simulation in `tmux`

  ```bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
- Run the SLAM node in new window of `tmux`

  ```bash
  ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
  ```
- Run the control tool in new window of `tmux`

  ```bash
  rqt_robot_steering
  ```

### Building Packages

```sh
docker attach ros2-cartographer-ws
cd /home/ros2-agv-essentials/cartographer_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build
```

> After the build process, make sure to source the `install/setup.bash` file.  
> Otherwise, ROS2 will not locate the executable files. You can open a new terminal to accomplish this.

### Multi LiDAR - Single Robot SLAM test

#### Simulation

- multi_lidar_desp package: Description of a robot with multiple LiDARs.
- multi_lidar_gazebo package: Gazebo simulation of the robot with robot state publisher.

  ```bash
  ros2 launch multi_lidar_gazebo multi_lidar_gazebo.launch.py
  ```

#### Run the SLAM node

- Run the cartographer SLAM node in new window of `tmux`
  ```bash
  ros2 launch cartographer_demo cartographer_demo.launch.py
  ```

- Run the control tool in new window of `tmux`
  ```bash
  rqt_robot_steering
  ```

#### References

- [Cartographer Demo](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)
