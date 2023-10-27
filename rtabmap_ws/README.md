# ROS2 RTAB-Map

### Run with docker

```bash
git clone https://github.com/j3soon/ros2-agv-essentials.git
```

```bash
cd ros2-agv-essentials/rtabmap_ws/docker
docker-compose pull
docker-compose up -d --build
```

### Simple test with gazebo

- Attach to the container
  ```sh
  docker attach ros2-rtabmap-ws
  cd /home/ros2-agv-essentials/rtabmap_ws
  ```
- Launch Gazebo with turtlebot3 in `tmux`
  ```bash
  export TURTLEBOT3_MODEL=waffle
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
- Run rtabmap LiDAR demo in a new `tmux` window
  ```bash
  ros2 launch rtabmap_demos turtlebot3_scan.launch.py
  ```

### Reference

- [RTAB-Map wiki](https://github.com/introlab/rtabmap/wiki)

### Existing issues

- `VTK` warning
  ```bash
  QVTKOpenGLWidget: Warning: In /build/vtk6-6.3.0+dfsg1/Rendering/OpenGL2/vtkOpenGLRenderWindow.cxx, line 781
  ...
  ```
  - It seems that the warning isn't a big deal. But it will interrupt debugging in the future.
  - Possible solution : set `VTK_LEGACY_REMOVE`, but it required to build from source.
      - Still not tested yet.
  - [Issue Reference](https://discourse.vtk.org/t/vtk-9-0-rc1/2916)