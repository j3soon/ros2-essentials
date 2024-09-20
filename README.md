# ROS 2 Essentials

[![tests](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/test-common.yaml?label=tests)](https://github.com/j3soon/ros2-essentials/actions/workflows/test-common.yaml)

A repo containing essential ROS2 Humble features for controlling Autonomous Mobile Robots (AMRs). Please setup an Ubuntu environment before using this repo.

> Please note that this repo is under rapid development. The code is not guaranteed to be stable, and breaking changes may occur.

## Pre-built Docker Images

The following Docker images can be pulled by `docker pull <IMAGE_NAME>` or by `docker compose pull` in the corresponding directory.

Pulling the pre-built Docker images can bypass the time-consuming building process (for both docker compose & devcontainers).

| Docker Image | amd64 | arm64 | Notes | Maintainer |
|--------------|-------|-------|-------|------------|
| `j3soon/ros2-template-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-template-ws/tags)][[code](./template_ws)] | ✔️ | ✔️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| `j3soon/ros2-orbslam3-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-orbslam3-ws/tags)][[code](./orbslam3_ws)] | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| `j3soon/ros2-rtabmap-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-rtabmap-ws/tags)][[code](./rtabmap_ws)] | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| `j3soon/ros2-ros1-bridge-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-ros1-bridge-ws/tags)][[code](./ros1_bridge_ws)] | ✔️ | ✔️ | Skip linting | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| `j3soon/ros2-ros1-bridge-build-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-ros1-bridge-build-ws/tags)][[code](./ros1_bridge_ws)] | ✔️ | ✔️ | Skip linting | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| `j3soon/ros2-cartographer-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-cartographer-ws/tags)][[code](./cartographer_ws)] | ✔️ | ✔️ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| `j3soon/ros2-husky-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-husky-ws/tags)][[code](./husky_ws)] | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| `j3soon/ros2-kobuki-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-kobuki-ws/tags)][[code](./kobuki_ws)] | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| `j3soon/ros2-vlp-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-vlp-ws/tags)][[code](./vlp_ws)] | ✔️ | ✔️ | Real-world support | [Assume Zhan](https://github.com/Assume-Zhan) |
| `j3soon/ros2-gazebo-world-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-gazebo-world-ws/tags)][[code](./gazebo_world_ws)] | ✔️ | ❌️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| `j3soon/aloha-ws` [[tags](https://hub.docker.com/r/j3soon/ros2-aloha-ws/tags)][[code](./aloha_ws)] | ✔️ | ❌️ | Simulation only | |

## Acknowledgement

The code is mainly contributed by [Johnson](https://github.com/j3soon), [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Assume Zhan](https://github.com/Assume-Zhan), [Lam Chon Hang](https://github.com/ClassLongJoe1112), and others. For a full list of contributors, please refer to the [contribution list](https://github.com/j3soon/ros2-essentials/graphs/contributors).

We extend our gratitude to [ElsaLab][elsalab] and [NVIDIA AI Technology Center (NVAITC)][nvaitc] for their support in making this project possible.

[elsalab]: https://github.com/elsa-lab
[nvaitc]: https://github.com/NVAITC

Disclaimer: this is not an official NVIDIA product.

## License

All modifications are licensed under [Apache License 2.0](./LICENSE).

However, this repository includes many dependencies released under different licenses. For information on these licenses, please check the commit history. Make sure to review the license of each dependency before using this repository.

> The licenses for dependencies will be clearly documented in the workspace README in the future.

## Supplementary

### Installing Docker

Follow [this post](https://tutorial.j3soon.com/docker/installation/) for the installation instructions.
