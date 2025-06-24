#!/bin/bash -e

# Ref: https://serverfault.com/a/160961
trap "echo Terminating...; exit;" SIGINT SIGTERM

images=(
    "osrf/ros:humble-desktop-full"
    "osrf/ros:jazzy-desktop-full"
    "osrf/ros:noetic-desktop-full"
    "j3soon/ros2-template-ws"
    "j3soon/ros2-orbslam3-ws"
    "j3soon/ros2-rtabmap-ws"
    "j3soon/ros2-ros1-bridge-ws"
    "j3soon/ros2-ros1-bridge-build-ws"
    "j3soon/ros2-cartographer-ws"
    "j3soon/ros2-husky-ws"
    "j3soon/ros2-kobuki-ws"
    "j3soon/ros2-vlp-ws"
    "j3soon/ros2-gazebo-world-ws"
    "j3soon/ros2-aloha-ws"
    "j3soon/ros2-turtlebot3-ws"
    "j3soon/ros2-delto-gripper-ws"
)

# Loop through each image and pull it
for image in "${images[@]}"
do
    echo "Pulling amd64 Docker image: $image"
    docker pull --platform=linux/amd64 "$image"
done

echo "Done."
