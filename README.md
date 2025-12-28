# ROS 2 Essentials

[![tests](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/test-common.yaml?label=tests)](https://github.com/j3soon/ros2-essentials/actions/workflows/test-common.yaml)
[![docs](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-docs.yaml?label=docs)](https://j3soon.github.io/ros2-essentials/)

A repo containing essential ROS2 Humble features for controlling Autonomous Mobile Robots (AMRs) and robotic arm manipulators. Please setup an Ubuntu environment before using this repo.

The goal of this repo is to allow seamless robot policy reuse between simulation and reality powered by [(Omniverse) Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html), [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/index.html), and [Isaac ROS](https://nvidia-isaac-ros.github.io/index.html). In general, the amd64 images support both simulation and real robot control, while the arm64 images only supports real robot control.

> Please note that this repo is under rapid development. The code is not guaranteed to be stable, and breaking changes may occur.

The documentation is hosted on <https://j3soon.github.io/ros2-essentials/>.

## System Requirements

| Use Case | Platform | Hardware | Software | Notes |
|----------|----------|----------|----------|-------|
| Simulation/Deployment | x86_64 | RTX GPU, 500GB+ SSD | Ubuntu 22.04, [NVIDIA Driver](https://ubuntu.com/server/docs/nvidia-drivers-installation), [Docker](https://docs.docker.com/engine/install/ubuntu/), [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) | See [this page](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) for more details. |
| Deployment-Only | Jetson | Jetson Orin, 500GB+ SSD | JetPack 6 | See [this page](https://nvidia-isaac-ros.github.io/getting_started/index.html) for more details.

Make sure to install the required software prerequisites before using this repo.

> The code may still work on lower-spec systems, such as those without GPUs or on older operating systems. However, these are not tested and may require manual adjustments. Proceed with caution.

## Setup

```sh
git clone https://github.com/j3soon/ros2-essentials.git
cd ros2-essentials
./scripts/post_install.sh
```

> Note that the `post_install.sh` script should be run after each change to the repository (such as switching to another branch or pulling the latest changes). In addition, the script can be run with the `-f` or `--force` flag to force removal of hard links if needed.

Then, configure the container user ID to match your host user ID by modifying the host `~/.bashrc` (or `~/.zshrc`) to include the following line:

```sh
export USER_UID=$(id -u)
```

This step is optional if you have user ID 1000 on host.

Next, choose a workspace from the table below and follow its documentation to get started. The rest of this document contains optional information. The search bar at the top right corner of the [documentation site](https://j3soon.github.io/ros2-essentials/) and [GitHub repository](https://github.com/j3soon/ros2-essentials) is a good way to navigate to the relevant documentation and commits.

## Updating the Repository

Make sure to run the `post_install.sh` script after pulling the latest changes to ensure that the hard links are properly set up.

```sh
git pull
./scripts/post_install.sh -f
```

## Pre-built Workspaces

Pre-built Docker images for each workspace can be pulled by running `docker compose pull` in the corresponding workspace directory. Pulling these images bypasses the time-consuming build process (for both Docker Compose and Dev Containers).

The docker image of the template workspace is share by most of the workspace, allowing saving spaces by sharing common packages. Click on the following workspaces to navigate to their respective documentation.

| Workspace | amd64 | arm64 | Notes | Maintainer |
|-----------|-------|-------|-------|------------|
| [Template](https://j3soon.github.io/ros2-essentials/template-ws/) | ✔️ | ✔️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Johnson Sun](https://github.com/j3soon) |
| [ORB-SLAM3](https://j3soon.github.io/ros2-essentials/orbslam3-ws) | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [RTAB-Map](https://j3soon.github.io/ros2-essentials/rtabmap-ws/) | ✔️ | ❌ | | [Assume Zhan](https://github.com/Assume-Zhan) |
| [ROS1 Bridge](https://j3soon.github.io/ros2-essentials/ros1-bridge-ws/) | ✔️ | ✔️ | Skip linting | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Clearpath Husky](https://j3soon.github.io/ros2-essentials/husky-ws/) | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Johnson Sun](https://github.com/j3soon) |
| [Yujin Robot Kobuki](https://j3soon.github.io/ros2-essentials/kobuki-ws/) | ✔️ | ✔️ | Real-world support | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Velodyne VLP-16](https://j3soon.github.io/ros2-essentials/vlp-ws/) | ✔️ | ✔️ | Real-world support | [Assume Zhan](https://github.com/Assume-Zhan) |
| [Gazebo World](https://j3soon.github.io/ros2-essentials/gazebo-world-ws/) | ✔️ | ❌️ | | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [ALOHA](https://j3soon.github.io/ros2-essentials/aloha-ws/) | ✔️ | ✔️ | Simulation only | [Johnson Sun](https://github.com/j3soon) |
| [Turtlebot3](https://j3soon.github.io/ros2-essentials/turtlebot3-ws/) | ✔️ | ❌️ | Simulation only | [Johnson Sun](https://github.com/j3soon) |
| [Tesollo Delto Gripper](https://j3soon.github.io/ros2-essentials/delto-gripper-ws/) | ✔️ | ❌️ | Simulation only | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Unitree Go2](https://j3soon.github.io/ros2-essentials/go2-ws/) | ✔️ | ❌️ | Simulation only | [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Assume Zhan](https://github.com/Assume-Zhan), [Johnson Sun](https://github.com/j3soon) |
| [Unitree H1](https://j3soon.github.io/ros2-essentials/h1-ws/) | ✔️ | ❌️ | Simulation only | [Johnson Sun](https://github.com/j3soon) |

If you have trouble using a workspace, please [open an issue](https://github.com/j3soon/ros2-essentials/issues) and tag the current maintainers mentioned above.

## Pre-configured Modules

Modules with `Default: ✔️` are installed by default in all workspaces.

Edit the `build.args` section in the `*_ws/docker/compose.yml` file and rebuild the workspace to add or remove modules.

| Module | amd64 | arm64 | Notes | Default | Maintainer |
|--------|-------|-------|-------|---------|------------|
| [ROS2](https://j3soon.github.io/ros2-essentials/docker-modules/ros2/) | ✔️ | ✔️ | ROS2 Humble | ✔️ | [Yu-Zhong Chen](https://github.com/YuZhong-Chen) |
| [Cartographer](https://j3soon.github.io/ros2-essentials/docker-modules/cartographer/) | ✔️ | ✔️ | ROS2 Cartographer | ➖ | [Assume Zhan](https://github.com/Assume-Zhan), [@yuhsiang1117](https://github.com/yuhsiang1117) |
| [CUDA Toolkit](https://j3soon.github.io/ros2-essentials/docker-modules/cuda-toolkit/) | ✔️ | ️TODO | CUDA 12.6 | ❌ | [Johnson Sun](https://github.com/j3soon) |
| [Isaac Sim](https://j3soon.github.io/ros2-essentials/docker-modules/isaac-sim/) | ✔️ | ❌ | Isaac Sim 5.0.0 Binary Install | ✔️ | [Johnson Sun](https://github.com/j3soon), [@JustinShih0918](https://github.com/JustinShih0918) |
| [Isaac Lab](https://j3soon.github.io/ros2-essentials/docker-modules/isaac-lab/) | ✔️ | ❌ | Isaac Lab 2.2.1 Git Install | ✔️ | [Johnson Sun](https://github.com/j3soon) |
| [Isaac ROS](https://j3soon.github.io/ros2-essentials/docker-modules/isaac-ros/) | ✔️ | TODO | Isaac ROS 3.2 Apt Install (Base only) | ❌ | [Johnson Sun](https://github.com/j3soon) |

## Docker Compose Cleanup

```sh
# cd into a workspace directory's docker directory
docker compose down --volumes --remove-orphans
docker volume rm ros2-gazebo-cache
docker volume rm ros2-isaac-sim-cache
docker volume rm ros2-isaac-ros-assets
```

## Building Documentation

```sh
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
# Create and activate python virtual environment
uv venv --python 3.10
source .venv/bin/activate
# Install dependencies and start serving
cd docs
uv pip install -r requirements.txt
# (Optional) Comment out the `git-committers` block in `docs/mkdocs.yml` to speed up build.
mkdocs serve
# Go to https://127.0.0.1:8000 to view the site.
```

## VSCode Intellisense

If you have installed [Isaac Sim 4.5.0](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html) to the default path `~/isaacsim`, you can simply enable IntelliSense for editing any Isaac Sim scripts by running the following script:

```sh
scripts/setup_isaac_link.sh
```

This is automatically done by running `./scripts/post_install.sh`.

## Reusing Docker Build Cache

The default `docker` build driver cannot pull pre-built caches from Docker Hub. Using the `docker-container` driver [enables cache reuse](https://docs.docker.com/build/builders/drivers/) but may add [a few minutes of overhead](https://github.com/docker/buildx/issues/107) for sending tarballs after the build. You should consider this tradeoff if you choose to switch the build driver.

### Switching to `docker-container` Build Driver

```sh
docker buildx ls
# Install buildx as `docker build` alias
docker buildx install
# Create and use new builder
docker buildx create --name docker-container --driver docker-container --driver-opt default-load=true --use
```

After setting this, using `docker compose build` will use the build cache from Docker Hub. If you want to switch back to the default `docker` build driver, follow the instructions below.

### Switching back to `docker` Build Driver

```sh
# Uninstall buildx as `docker build` alias
docker buildx uninstall
# Unuse the created builder and remove it
docker buildx use default
docker buildx stop docker-container
docker buildx rm -f --all-inactive
```

## Developer Notes

### GitHub Actions

The GitHub Actions workflow is designed to share build caches between workspaces efficiently. The template workspace is built first, and its cache is then reused by other workspaces. This means that while the template workspace build appears in the commit history, other workspace builds are triggered indirectly and only show up in the GitHub Actions tab. For implementation details, see [commit `024f52a`](https://github.com/j3soon/ros2-essentials/commit/024f52a2bb8a58ad20c03a067560215e8cef6307).

Some current CI builds are flaky and may require re-running.

## Acknowledgement

The code is mainly contributed by [Johnson Sun](https://github.com/j3soon), [Yu-Zhong Chen](https://github.com/YuZhong-Chen), [Assume Zhan](https://github.com/Assume-Zhan), and others. For a full list of contributors, please refer to the [contribution list](https://github.com/j3soon/ros2-essentials/graphs/contributors).

We extend our gratitude to [ElsaLab][elsalab] and [NVIDIA AI Technology Center (NVAITC)][nvaitc] for their support in making this project possible.

[elsalab]: https://github.com/elsa-lab
[nvaitc]: https://github.com/NVAITC

Disclaimer: this is not an official NVIDIA product.

## License

All modifications are licensed under [Apache License 2.0](https://github.com/j3soon/ros2-essentials/blob/main/LICENSE).

However, this repository includes many dependencies released under different licenses. For information on these licenses, please check the commit history. Make sure to review the license of each dependency before using this repository.

The detailed license information is documented [here](https://j3soon.github.io/ros2-essentials/dependencies/).

## Supplementary

### Installing Docker

Follow [this post](https://tutorial.j3soon.com/docker/installation/) for the installation instructions.

### Installing NVIDIA Container Toolkit

Follow [this post](https://tutorial.j3soon.com/docker/nvidia-gpu-support/) for the installation instructions.
