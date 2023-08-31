# ROS2 ORB-SLAM3

A repo build a docker image with ros2 humble and orb-slam3

### Run with dockerfile

```bash
git clone https://github.com/Assume-Zhan/ROS2-SLAM3.git ros2_orbslam_docker
```

```bash
cd ros2_orbslam_docker
./run.sh
```

### Use current complete image

```bash
docker pull assume/humble_orbslam3
```

### Simple Test With Dataset

- Download dataset
```bash
wget -P ~/ http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
```
- ROS1 bag and ROS2 bag conversion [reference](https://docs.openvins.com/dev-ros1-to-ros2.html)
```bash
pip3 install rosbags
rosbags-convert ~/V1_02_medium.bag
```
- Run the bag file
```bash
ros2 bag play V1_02_medium/V1_02_medium.db3 --remap /cam0/image_raw:=/camera
```
- Run the ORB-SLAM3
```bash
ros2 run orbslam3 mono ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples_old/Monocular/EuRoC.yaml false
```

#### Reference repo or issues

[Solve build failure](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/566)

[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

[SLAM2 and Foxy docker](https://github.com/alsora/ros2-ORB_SLAM2/tree/master)

[SLAM3 and Foxy](https://github.com/alsora/ros2-ORB_SLAM2/tree/master)

[Error when using humble](https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970)