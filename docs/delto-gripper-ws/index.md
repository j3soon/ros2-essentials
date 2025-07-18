# Tesollo Delto Gripper

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/delto_gripper_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-delto-gripper-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-delto-gripper-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=delto_gripper_ws)](https://github.com/j3soon/ros2-essentials/commits/main/delto_gripper_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--delto--gripper--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-delto-gripper-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-delto-gripper-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-delto-gripper-ws)

This repository will help you configure the environment for Delto Gripper quickly.

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/delto_gripper_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may replace the `docker compose build` command with `docker compose pull`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-delto-gripper-ws bash
```

If the initial build somehow failed, run:

```sh
rm -r build install
colcon build --symlink-install
```

Once you have finished testing, you can stop and remove the container with:

```sh
docker compose down
```

## Testing

### Launch the Isaacsim

```bash
~/isaacsim/isaac-sim.sh
```

> After the Isaacsim is launched, open the delto gripper usd file and play the simulation.  
> It is located at: `delto_gripper_ws/src/DELTO_M_ROS2/dg_isaacsim/dg5f_right/dg5f_right.usd`

### Launch the Delto Gripper Demo

```bash
ros2 launch dg5f_isaacsim dg5f_right_isaacsim.launch.py
```

<video controls>
  <source src="assets/preview-video.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## References

Dependencies:

- [Tesollo-Delto/DELTO_M_ROS2](https://github.com/Tesollo-Delto/DELTO_M_ROS2) (at commit 942c166) is licensed under the BSD-3-Clause license.
