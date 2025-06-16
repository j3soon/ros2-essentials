# Tesollo Delto Gripper

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/delto_gripper_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-delto-gripper-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-delto-gripper-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=delto_gripper_ws)](https://github.com/j3soon/ros2-essentials/commits/main/delto_gripper_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--delto--gripper--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-delto-gripper-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-delto-gripper-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-delto-gripper-ws)

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

## References

Dependencies:

- [Tesollo-Delto/DELTO_M_ROS2](https://github.com/Tesollo-Delto/DELTO_M_ROS2) (at commit 942c166) is licensed under the BSD-3-Clause license.
