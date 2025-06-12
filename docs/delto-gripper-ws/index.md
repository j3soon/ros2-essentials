# delto_gripper_ws

This repository will help you configure the environment for Delto Gripper quickly.

## Run the container

```bash
# Run the container
cd /home/ros2-essentials/delto_gripper_ws/docker
docker compose up -d --build
docker exec -it ros2-delto-gripper-ws bash
```

## Building Packages

> Normally, when you enter the container, the packages will be built automatically.

```bash
cd /home/ros2-essentials/delto_gripper_ws
colcon build --symlink-install
```

## Testing

### Launch the Isaacsim

```bash
isaacsim omni.isaac.sim
```

> After the Isaacsim is launched, open the delto gripper usd file and play the simulation.  
> It is located at: `delto_gripper_ws/src/DELTO_M_ROS2/dg_isaacsim/dg5f_right/dg5f_right.usd`

### Launch the Delto Gripper Demo

```bash
ros2 launch dg5f_isaacsim dg5f_right_isaacsim.launch.py
```
