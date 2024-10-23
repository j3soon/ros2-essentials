# ROS 2 Essentials

[![tests](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/test-common.yaml?label=tests)](https://github.com/j3soon/ros2-essentials/actions/workflows/test-common.yaml)
[![docs](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-docs.yaml?label=docs)](https://j3soon.github.io/ros2-essentials/)

A repo containing essential ROS2 Humble features for controlling Autonomous Mobile Robots (AMRs). Please setup an Ubuntu environment before using this repo.

> Please note that this repo is under rapid development. The code is not guaranteed to be stable, and breaking changes may occur.

The documentation is hosted on <https://j3soon.github.io/ros2-essentials/>.

## Pre-built Workspaces

Pre-built Docker images for each workspace can be pulled by running `docker compose pull` in the corresponding workspace directory.

Pulling the pre-built Docker images can bypass the time-consuming building process (for both docker compose & devcontainers).

| Workspace | amd64 | arm64 | Notes | Maintainer |
|-----------|-------|-------|-------|------------|
| [Template](https://j3soon.github.io/ros2-essentials/template-ws/) | ✔️ | ✔️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [ORB-SLAM3](https://j3soon.github.io/ros2-essentials/orbslam3_ws) | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [RTAB-Map](https://j3soon.github.io/ros2-essentials/rtabmap_ws/) | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [ROS1 Bridge](https://j3soon.github.io/ros2-essentials/ros1_bridge_ws/) | ✔️ | ✔️ | Skip linting | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Cartographer](https://j3soon.github.io/ros2-essentials/cartographer_ws/) | ✔️ | ✔️ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [Clearpath Husky](https://j3soon.github.io/ros2-essentials/husky_ws/) | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Yujin Robot Kobuki](https://j3soon.github.io/ros2-essentials/kobuki_ws/) | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Velodyne VLP-16](https://j3soon.github.io/ros2-essentials/vlp_ws/) | ✔️ | ✔️ | Real-world support | [Assume Zhan](https://github.com/Assume-Zhan) |
| [Gazebo World](https://j3soon.github.io/ros2-essentials/gazebo_world_ws/) | ✔️ | ❌️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [ALOHA](https://j3soon.github.io/ros2-essentials/aloha_ws/) | ✔️ | ✔️ | Simulation only | |

## Building Documentation

```sh
virtualenv venv -p python3
source venv/bin/activate
cd docs
pip install -r requirements.txt
mkdocs serve
# Go to https://127.0.0.1:8000 to view the site.
```

## Acknowledgement

The code is mainly contributed by [Johnson](https://github.com/j3soon), [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Assume Zhan](https://github.com/Assume-Zhan), [Lam Chon Hang](https://github.com/ClassLongJoe1112), and others. For a full list of contributors, please refer to the [contribution list](https://github.com/j3soon/ros2-essentials/graphs/contributors).

We extend our gratitude to [ElsaLab][elsalab] and [NVIDIA AI Technology Center (NVAITC)][nvaitc] for their support in making this project possible.

[elsalab]: https://github.com/elsa-lab
[nvaitc]: https://github.com/NVAITC

Disclaimer: this is not an official NVIDIA product.

## License

All modifications are licensed under [Apache License 2.0](https://github.com/j3soon/ros2-essentials/blob/main/LICENSE).

However, this repository includes many dependencies released under different licenses. For information on these licenses, please check the commit history. Make sure to review the license of each dependency before using this repository.

> The licenses for dependencies will be clearly documented in the workspace README in the future.

## Supplementary

### Installing Docker

Follow [this post](https://tutorial.j3soon.com/docker/installation/) for the installation instructions.

### Installing NVIDIA Container Toolkit

Follow [this post](https://tutorial.j3soon.com/docker/nvidia-gpu-support/) for the installation instructions.
