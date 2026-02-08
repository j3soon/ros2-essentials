# ORB-SLAM3

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/orbslam3_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-orbslam3-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-orbslam3-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=orbslam3_ws)](https://github.com/j3soon/ros2-essentials/commits/main/orbslam3_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--orbslam3--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-orbslam3-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-orbslam3-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-orbslam3-ws)

> Last tested on TODO.

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/orbslam3_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may replace the `docker compose build` command with `docker compose pull`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-orbslam3-ws bash
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

## Simple Test With Dataset

- Attach to the container
  ```sh
  docker attach ros2-orbslam3-ws
  cd /home/ros2-essentials/orbslam3_ws
  ```
- Prepare data, only need to be done once
  - Download dataset (~1.2G)
    ```bash
    wget -P . http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
    ```
  - ROS1 bag and ROS2 bag conversion [reference](https://docs.openvins.com/dev-ros1-to-ros2.html)
    ```bash
    sudo pip3 install rosbags
    rosbags-convert ./V1_02_medium.bag
    ```
- Play the bag file in `tmux`
  ```bash
  ros2 bag play V1_02_medium/V1_02_medium.db3 --remap /cam0/image_raw:=/camera
  ```
- Run the ORB-SLAM3 in a new `tmux` window
  ```bash
  source ~/test_ws/install/local_setup.bash
  ros2 run orbslam3 mono ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples_old/Monocular/EuRoC.yaml false
  ```
  2 windows will pop up, showing the results.

## Reference repo or issues

- [Solve build failure](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/566)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [SLAM2 and Foxy docker](https://github.com/alsora/ros2-ORB_SLAM2/tree/master)
- [Error when using humble](https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970)
