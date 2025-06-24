#!/bin/bash -e

# Ref: https://serverfault.com/a/160961
trap "echo Terminating...; exit;" SIGINT SIGTERM

images=(
    "ros:humble"
    "ros:jazzy"
    "ros:noetic"
    "j3soon/ros2-template-ws"
    "j3soon/ros2-ros1-bridge-ws"
    "j3soon/ros2-ros1-bridge-build-ws"
    "j3soon/ros2-cartographer-ws"
    "j3soon/ros2-husky-ws"
    "j3soon/ros2-kobuki-ws"
    "j3soon/ros2-vlp-ws"
    "j3soon/ros2-aloha-ws"
)

# Loop through each image and pull it
for image in "${images[@]}"
do
    echo "Pulling arm64 Docker image: $image"
    docker pull --platform=linux/arm64 "$image"
done

echo "Done."
