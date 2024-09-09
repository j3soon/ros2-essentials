# aloha_ws

## Start Container

```sh
cd ~/ros2-essentials/aloha_ws/docker
docker compose up
# in a new terminal
docker exec -it ros2-aloha-ws bash
```

The commands in the following sections assume that you are inside the Docker container.

## View Robot Model in RViz

```sh
ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=vx300s use_joint_pub_gui:=true
```

It is worth noting that `aloha_vx300s.urdf.xacro` and `vx300s.urdf.xacro` files are identical. We opt to use `vx300s` since `aloha_vx300s` seems to lack corresponding configs, such as those for MoveIt 2.

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

## View Robot Model in Isaac Sim

Prepare USD files:

```sh
cd /home/ros2-essentials/aloha_ws/isaacsim/scripts
./create_urdf_from_xacro.sh
python3 create_vx300s_from_urdf.py
python3 create_vx300s_with_omnigraph.py
```

Launch Isaac Sim app:

```sh
isaacsim omni.isaac.sim
```

Open pre-configured USD file with OmniGraph:

- `File > Open` and in `File name:` type:
  ```
  /home/ros2-essentials/aloha_ws/isaacsim/assets/vx300s_og.usd
  ```
- Click `Window > Visual Scripting > Action Graph`
- In the `Action Graph` tab, click `Edit Action Graph` and select `/ActionGraph`
- Click `Play (SPACE)`

View the current joint states:

```sh
# in a new terminal
docker exec -it ros2-aloha-ws bash
ros2 topic echo /joint_states
```

Set the current joint states:

(TODO)

## References

- [Interbotix X-Series Arms \| Trossen Robotics Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/index.html)
  - [ROS 2 Interface](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2.html)
    - [ROS 2 Standard Software Setup](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
  - [ROS 2 Open Source Packages](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages.html)
- [Stationary ALOHA Software Setup \| Trossen Robotics Documentation](https://docs.trossenrobotics.com/aloha_docs/getting_started/stationary/software_setup.html)
