# ROS 2 Essentials

A repo containing essential ROS2 Humble features for controlling Autonomous Mobile Robots (AMRs). Please setup an Ubuntu environment before using this repo.

## Pre-built Docker Images

The following Docker images can be pulled by `docker pull <IMAGE_NAME>` or by `docker compose pull` in the corresponding directory.

Pulling the pre-built Docker images can bypass the time-consuming building process (for both docker compose & devcontainers).

| Docker Image | Description | amd64 | arm64 |
|--------------|-------------|-------|-------|
| [j3soon/ros2-template-ws](https://hub.docker.com/r/j3soon/ros2-template-ws/tags) | [`./template_ws`](./template_ws) | ✔️ | ❌ |
| [j3soon/ros2-orbslam3-ws](https://hub.docker.com/r/j3soon/ros2-orbslam3-ws/tags) | [`./orbslam3_ws`](./orbslam3_ws) | ✔️ | ❌ |
| [j3soon/ros2-rtabmap-ws](https://hub.docker.com/r/j3soon/ros2-rtabmap-ws/tags) | [`./rtabmap_ws`](./rtabmap_ws) | ✔️ | ❌ |
| [j3soon/ros2-ros1-bridge-ws](https://hub.docker.com/r/j3soon/ros2-ros1-bridge-ws/tags) | [`./ros1_bridge_ws`](./ros1_bridge_ws) | ✔️ | ❌ |
| [j3soon/ros2-ros1-bridge-build-ws](https://hub.docker.com/r/j3soon/ros2-ros1-bridge-build-ws/tags) | [`./ros1_bridge_ws`](./ros1_bridge_ws) | ✔️ | ❌ |
| [j3soon/ros2-cartographer-ws](https://hub.docker.com/r/j3soon/ros2-cartographer-ws/tags) | [`./cartographer_ws`](./cartographer_ws) | ✔️ | ❌ |
| [j3soon/ros2-husky-ws](https://hub.docker.com/r/j3soon/ros2-husky-ws/tags) | [`./husky_ws`](./husky_ws) | ✔️ | ✔️ |
| [j3soon/ros2-kobuki-ws](https://hub.docker.com/r/j3soon/ros2-kobuki-ws/tags) | [`./kobuki_ws`](./kobuki_ws) | ✔️ | ✔️ |
| [j3soon/ros2-vlp-ws](https://hub.docker.com/r/j3soon/ros2-vlp-ws/tags) | [`./vlp_ws`](./vlp_ws) | ✔️ | ✔️ |

## Contributors

The code is mainly contributed by [Johnson](https://github.com/j3soon), [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Assume Zhan](https://github.com/Assume-Zhan), and [others](https://github.com/j3soon/ros2-essentials/graphs/contributors).

## License

All modifications are licensed under [Apache License 2.0](./LICENSE).

However, this repository includes many dependencies released under different licenses. For information on these licenses, please check the commit history. Make sure to review the license of each dependency before using this repository.

> The licenses for dependencies will be clearly documented in the workspace README in the future.

## Supplementary

### Installing Docker

Follow [this post](https://tutorial.j3soon.com/docker/installation/) for the installation instructions.
