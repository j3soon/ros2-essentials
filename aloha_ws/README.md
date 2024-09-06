# aloha_ws

## Start Container

```sh
cd ~/ros2-essentials/aloha_ws/docker
docker compose up
# in a new terminal
docker exec -it ros2-aloha-ws bash
```

## View Robot Model in RViz

```sh
ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=vx300s use_joint_pub_gui:=true
```

## View Robot Model in Gazebo

```sh
ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=vx300s
```

## MoveIt 2

```sh
# with Gazebo
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=vx300s hardware_type:=gz_classic
# or without Gazebo
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=vx300s hardware_type:=fake
```

## References

- [Interbotix X-Series Arms \| Trossen Robotics Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/index.html)
  - [ROS 2 Interface](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2.html)
    - [ROS 2 Standard Software Setup](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
  - [ROS 2 Open Source Packages](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages.html)
- [Stationary ALOHA Software Setup \| Trossen Robotics Documentation](https://docs.trossenrobotics.com/aloha_docs/getting_started/stationary/software_setup.html)