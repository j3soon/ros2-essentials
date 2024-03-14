# citysim

This package is derived from the [osrf/citysim](https://github.com/osrf/citysim) repository.

## Testing

### Using command

Using the command below, you can launch Gazebo without ROS 2.  
Please ensure that you have set up the `GAZEBO_MODEL_PATH` correctly.

```bash
cd $ROS2_WS/src/citysim
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ros2-agv-essentials/husky_ws/src/citysim/models
gazebo worlds/simple_city.world
```

### Using ROS2

```bash
cd $ROS2_WS
colcon build
ros2 launch citysim simple_world_launch.py
```
