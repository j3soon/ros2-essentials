# ros1_bridge_ws

This workspace is utilized to create a bridge between ROS1 and ROS2-humble.

# 🌱 Introduction 🌱

ros1_bridge provides a network bridge that enables the exchange of messages between ROS 1 and ROS 2.  
You can locate the original repository [here](https://github.com/ros2/ros1_bridge).

Within this workspace, you'll find a Dockerfile specifically crafted to build both ros-humble and ros1_bridge from their source code.  
This necessity arises due to a version conflict between ```catkin-pkg-modules``` available in the Ubuntu repository and the one in the ROS. 

[The official explanation](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html#ros-2-via-debian-packages)

---

Assuming you are already familiar with the ROS network architecture.  
If not, I recommend reading the tutorial provided below first.  

- https://wiki.ros.org/Master
- https://docs.ros.org/en/humble/Concepts/Basic/About-Discovery.html

# 🚩 How to use 🚩

## 1. Get the docker image

While it is possible to build the image directly from the Dockerfile, it may not be the most efficient choice. 

Compiling ros-humble from source can be time-consuming, and to save time, I've already uploaded the image to Dockerhub.  
You can find the repository  [here](https://hub.docker.com/r/yuzhong1214/ros1_bridge).

If you still prefer to build the image yourself, you can open the workspace in VScode,  
press ```F1```, and enter ```> Dev Containers: Rebuild Container```. 

Please note that this process may take approximately 1 hour to complete.

## 2. Run the container

If you build the image through the devcontainer,  
it will automatically start the container so that you can skip this step entirely.

To manually run the container using the command line, you can use the following command:

```bash=
docker run \
    -it \
    --rm \
    --network=host \
    -v /dev/shm:/dev/shm \
    yuzhong1214/ros1_bridge:min
```

> ⚠️ Mount /dev/shm into container ⚠️
> 
> The latest releases of Fast-DDS come with the SharedMemory transport enabled by default.  
> So, it's very important for you to mount shared memory into every container you intend to communicate with when using Fast-DDS.  
> This ensures proper communication between containers.
> 
> Reference : https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676

## 3. Launch rosmaster in ROS1

As mentioned in https://github.com/ros2/ros1_bridge/issues/391, you should avoid using ```roscore``` in ROS1.  
Instead, use ```rosmaster --core``` as an alternative.

## 4. Start the bridge

Please execute the following command in the terminal.

```bash=
./start_bridge.sh
```

If you are not using devcontainer, you can skip this step entirely.

## 5. Begin communication

You have successfully executed all the instructions.  
Now, you can proceed to initiate communication between ROS1 and ROS2-humble.

Please keep in mind that the bridge will be established only when  
there are matching publisher-subscriber pairs active for a topic on either side of the bridge.

# 🔍 Troubleshooting 🔍

## Failed to contact master

Before launching the master, make sure you have set the ```ROS_MASTER_URI```.  
Execute the following command to configure it.

```bash=
export ROS_MASTER_URI=http://localhost:11311
```

You can replace localhost with the actual IP address or hostname of your ROS Master.  
This command ensures that your ROS nodes know where to find the ROS Master for communication.

If you've modified the default URI, please execute the ```start_bridge.sh``` script with the new URI address  
to ensure proper configuration and communication.

For example, if you've changed the default URI to ```127.0.0.1```, you can run the ```start_bridge.sh``` script like this :

```bash=
./start_bridge.sh http://127.0.0.1:11311
```

If you are running the container using the command-line interface (CLI), you can use the following command :

```bash=
docker run \
    -it \
    --rm \
    --network=host \
    -v /dev/shm:/dev/shm \
    yuzhong1214/ros1_bridge:min \
    ./start_bridge.sh http://127.0.0.1:11311
```
