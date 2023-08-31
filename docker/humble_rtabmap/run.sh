#!/bin/bash

DOCKERFILE=Dockerfile
IMAGE_NAME=rtabmap/humble
CONTAINER_NAME=test

# remove old containers
docker ps -a | grep $IMAGE_NAME | awk '{print $1}' | xargs -r docker rm

# remove old images
docker rmi $IMAGE_NAME

# build docker
docker build -t $IMAGE_NAME -f $DOCKERFILE .

# run docker
docker run \
    -it --rm \
    --gpus all \
    -h $(hostname) $2 \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/tmp/xauth \
    -v ~/.Xauthority:/tmp/xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    --name  $CONTAINER_NAME \
    $IMAGE_NAME
	
