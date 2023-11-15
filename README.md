# ROS 2 AGV Essentials

A repo containing essential ROS2 Humble features for controlling Autonomous Guided Vehicles (AGVs). Please setup an Ubuntu environment before using this repo.

## Pre-built Docker Images

The following Docker images can be pulled by `docker pull <IMAGE_NAME>` or by `docker-compose pull` in the corresponding directory.

Pulling the pre-built Docker images can bypass the time-consuming building process (for both docker-compose & devcontainers).

| Docker Image | Description | Require `nvidia-docker`? |
|--------------|-------------|--------------------------|
| [j3soon/ros2-template-ws](https://hub.docker.com/repository/docker/j3soon/ros2-template-ws/tags) | [`./template_ws`](./template_ws) | No |
| [j3soon/ros2-orbslam3-ws](https://hub.docker.com/repository/docker/j3soon/ros2-orbslam3-ws/tags) | [`./orbslam3_ws`](./orbslam3_ws) | No |
| [j3soon/ros2-ros1-bridge-ws](https://hub.docker.com/repository/docker/j3soon/ros2-ros1-bridge-ws/tags) | [`./ros1_bridge_ws`](./ros1_bridge_ws) | No |

## Supplementary

### Installing Docker

```sh
sudo apt-get install -y docker.io
sudo usermod -aG docker $USER # Add user to docker group
# Please re-login or restart to update user group information
```

We prefer `docker.io` over `docker-ce`, see [this post](https://stackoverflow.com/a/57678382) for more information. Using `docker-ce` should be fine as well though.

### Installing NVIDIA Docker

```sh
sudo apt-get install -y curl
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo apt-get install -y nvidia-container-runtime
sudo apt-get install -y nvidia-container-toolkit
```

The above commands are modified based on the instructions in [this post](http://web.archive.org/web/20230627162323/https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

The latest installation instructions suggest only installing `nvidia-container-toolkit`, but I haven't tested it yet, see [this post](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for further information.

For the difference between `nvidia-docker2`, `nvidia-container-runtime`, `nvidia-container-toolkit`, `libnvidia-container`, see [this comment](https://github.com/NVIDIA/nvidia-docker/issues/1268#issuecomment-632692949).
