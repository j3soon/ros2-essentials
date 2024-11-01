# ROS 2 Essentials

[![tests](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/test-common.yaml?label=tests)](https://github.com/j3soon/ros2-essentials/actions/workflows/test-common.yaml)
[![docs](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-docs.yaml?label=docs)](https://j3soon.github.io/ros2-essentials/)

A repo containing essential ROS2 Humble features for controlling Autonomous Mobile Robots (AMRs) and robotic arm manipulators. Please setup an Ubuntu environment before using this repo.

The goal of this repo is to allow seamless robot policy reuse between simulation and reality powered by [Omniverse Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) and [Isaac ROS](https://nvidia-isaac-ros.github.io/index.html). In general, the amd64 images support both simulation and real robot control, while the arm64 images only supports real robot control.

> Please note that this repo is under rapid development. The code is not guaranteed to be stable, and breaking changes may occur.

The documentation is hosted on <https://j3soon.github.io/ros2-essentials/>.

## Cloning the Repository

```sh
git clone https://github.com/j3soon/ros2-essentials.git
cd ros2-essentials
```

## Pre-built Workspaces

Pre-built Docker images for each workspace can be pulled by running `docker compose pull` in the corresponding workspace directory. Pulling these images bypasses the time-consuming build process (for both Docker Compose and Dev Containers).

The docker image of the template workspace is share by most of the workspace, allowing saving spaces by sharing common packages. Click on the following workspaces to navigate to their respective documentation.

| Workspace | amd64 | arm64 | Notes | Maintainer |
|-----------|-------|-------|-------|------------|
| [Template](https://j3soon.github.io/ros2-essentials/template-ws/) | ✔️ | ✔️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Johnson](https://github.com/j3soon) |
| [ORB-SLAM3](https://j3soon.github.io/ros2-essentials/orbslam3-ws) | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [RTAB-Map](https://j3soon.github.io/ros2-essentials/rtabmap-ws/) | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [ROS1 Bridge](https://j3soon.github.io/ros2-essentials/ros1-bridge-ws/) | ✔️ | ✔️ | Skip linting | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Cartographer](https://j3soon.github.io/ros2-essentials/cartographer-ws/) | ✔️ | ✔️ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [Clearpath Husky](https://j3soon.github.io/ros2-essentials/husky-ws/) | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Johnson](https://github.com/j3soon) |
| [Yujin Robot Kobuki](https://j3soon.github.io/ros2-essentials/kobuki-ws/) | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Velodyne VLP-16](https://j3soon.github.io/ros2-essentials/vlp-ws/) | ✔️ | ✔️ | Real-world support | [Assume Zhan](https://github.com/Assume-Zhan) |
| [Gazebo World](https://j3soon.github.io/ros2-essentials/gazebo-world-ws/) | ✔️ | ❌️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [ALOHA](https://j3soon.github.io/ros2-essentials/aloha-ws/) | ✔️ | ✔️ | Simulation only | [Johnson](https://github.com/j3soon) |

If you have trouble using a workspace, please [open an issue](https://github.com/j3soon/ros2-essentials/issues) and tag the current maintainers mentioned above.

## System Requirements

| Use Case | Platform | Hardware | Software | Notes |
|----------|----------|----------|----------|-------|
| Simulation/Deployment | x86_64 | RTX GPU, 500GB+ SSD | Ubuntu 22.04, [NVIDIA Driver](https://ubuntu.com/server/docs/nvidia-drivers-installation), [Docker](https://docs.docker.com/engine/install/ubuntu/), [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) | See [this page](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements) for more details. |
| Deployment-Only | Jetson | Jetson Orin, 500GB+ SSD | JetPack 6.0 | See [this page](https://nvidia-isaac-ros.github.io/getting_started/index.html) for more details.

Some functionalities may still work on lower-spec systems, such as those without GPUs or on operating systems other than Ubuntu 22.04. However, these configurations are not officially supported and may require manual adjustments. Use them with caution.

## Building Documentation

```sh
# (Optional) Create and activate python virtual environment
virtualenv venv -p python3
source venv/bin/activate
# Install dependencies and start serving
cd docs
pip install -r requirements.txt
mkdocs serve
# Go to https://127.0.0.1:8000 to view the site.
```

## Linking the README

```sh
rm docs/index.md
ln "${PWD}/README.md" docs/index.md
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
