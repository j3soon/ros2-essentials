# ORB-SLAM3

### Run with docker

```bash
git clone https://github.com/j3soon/ros2-essentials.git
```

```bash
cd ros2-essentials/orbslam3_ws/docker
docker compose pull
docker compose up -d --build
```

### Simple Test With Dataset

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

#### Reference repo or issues

- [Solve build failure](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/566)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [SLAM2 and Foxy docker](https://github.com/alsora/ros2-ORB_SLAM2/tree/master)
- [Error when using humble](https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970)
