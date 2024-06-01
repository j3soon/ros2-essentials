# ros1_bridge_ws

This workspace is utilized to create a bridge between ROS1 and ROS2-humble.

# ◻️ Introduction ◻️

ros1_bridge provides a network bridge that enables the exchange of messages between ROS 1 and ROS 2.  
You can locate the original repository [here](https://github.com/ros2/ros1_bridge).

Within this workspace, you'll find a Dockerfile specifically crafted to build both ros-humble and ros1_bridge from their source code.  
This necessity arises due to a version conflict between the `catkin-pkg-modules` available in the Ubuntu repository and the one in the ROS.

[The official explanation](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html#ros-2-via-debian-packages)

---

Assuming you are already familiar with the ROS network architecture.  
If not, I recommend reading the tutorial provided below first.  

- https://wiki.ros.org/Master
- https://docs.ros.org/en/humble/Concepts/Basic/About-Discovery.html

## 🌱 Structure 🌱

Here is the structure of this repo:

```
ros1_bridge_ws
├── .devcontainer
|   └── devcontainer.json
├── docker
|   ├── compose.yaml
|   ├── compose.debug.yaml
|   ├── Dockerfile
|   └── start-bridge.sh
└── README.md
```

# 🚩 How to use 🚩

> There are two services in the docker-compose, one is `ros1-bridge`, and the other one is `ros1-bridge-build`.  
> `ros1-bridge` should be sufficient for normal usage.  
> `ros1-bridge-build`, which contains all the necessary build tools, is used for debugging purposes.  
>
> If you are not debugging ros1-bridge, it is recommended to use the terminal rather than VScode-devcontainer.  
> By default, the VScode-devcontainer uses the `ros1-bridge-build` service.

## 1. Get the docker image

While building the image directly from the Dockerfile is possible, it may not be the most efficient choice.  
To save time, you can pull the image from Dockerhub instead of compiling it from the source:

If you still prefer to build the image yourself, please follow the instructions below:

- VScode user
  - Open the workspace in VScode, press `F1`, and enter `> Dev Containers: Rebuild Container`.
- Terminal user
  - Open a terminal, change the directory to the docker folder, and type `docker compose build`.

> Just so you know, the building process may take approximately 1 hour to complete.

## 2. Start the container

- VScode user
  - If you build the image through the devcontainer, it will automatically start the container.  
    After you get into the container, type `./start-bridge.sh` in the terminal.
- Terminal user
  - Open a terminal, change the directory to the docker folder, and type `docker compose up`.

## 3. Launch rosmaster in ROS1

As mentioned in https://github.com/ros2/ros1_bridge/issues/391, you should avoid using `roscore` in ROS1 to prevent the issue of not bridging `/rosout`.  
Instead, use `rosmaster --core` as an alternative.

This command is automatically executed when you run `docker compose up`.

## 4. Begin communication

You have successfully executed all the instructions.  
Now, you can proceed to initiate communication between ROS1 and ROS2-humble.

Please keep in mind that the bridge will be established only when  
there are matching publisher-subscriber pairs active for a topic on either side of the bridge.

# ✨ Example ✨

## Run the bridge and the example talker and listener

> Before beginning the example, ensure you have three containers ready:
>
> - `ros1_bridge`
> - `ROS1`
> - `ROS2`
>
> All containers are automatically launched when you run `docker compose up`.
>
> Furthermore, ensure that you mount `/dev/shm` into both the `ros1_bridge` and `ROS2` containers,  
> and that all containers share the host network.

### 1. Start the rosmaster in `ROS1` container

> This container is automatically launched when you run `docker compose up`. No further action is required.

```bash
# In ROS 1 container
rosmaster --core &
```

### 2. Start the `ros1_bridge` container

> This container is automatically launched when you run `docker compose up`. No further action is required.

(Same as [here](#2-start-the-container))

- VScode user
  - After getting into the container, type `./start-bridge.sh` in the terminal.
- Terminal user
  - ```bash
    # In docker folder
    docker compose run ros1-bridge
    ```

### 3. Run the talker and listener node

> These containers are automatically launched when you run `docker compose up`. You only need to execeute the commands below.

Run the listener node in the `ROS1` container and the talker node in the `ROS2` container.

```bash
docker exec -it ros1 /ros_entrypoint.sh bash
# In ROS 1 container
rosrun roscpp_tutorials listener
# or
# rosrun roscpp_tutorials talker
```

```bash
docker exec -it ros2 /ros_entrypoint.sh bash
# In ROS 2 container
# Use the same UID as ros1_bridge to prevent Fast-DDS Shared Memory permission issues.
# Ref: https://github.com/j3soon/ros2-essentials/pull/9#issuecomment-1795743063
useradd -ms /bin/bash user
su user
source /ros_entrypoint.sh
ros2 run demo_nodes_cpp talker
# or
# ros2 run demo_nodes_cpp listener
```

> You can run the talker node in `ROS1` and the listener node in `ROS2` if you'd like.  
> To achieve this, simply modify the command provided above.

Certainly, you can try the [example](https://github.com/ros2/ros1_bridge#example-1-run-the-bridge-and-the-example-talker-and-listener) provided by `ros1_bridge`.  
However, there's no need to source the setup script within the `ros1_bridge` container,  
simply starting the container will suffice.

# 🔍 Troubleshooting 🔍

> If you are trying to debug ros1_bridge, it is recommended to use the `ros1-bridge-build` service in docker-compose.  
> It contains all the necessary build tools, which should be helpful for you.

## Failed to contact master

Before launching the master, make sure you have set the `ROS_MASTER_URI`.  
Execute the following command to configure it.

```bash
export ROS_MASTER_URI=http://localhost:11311
```

You can replace localhost with the actual IP address or hostname of your ROS Master.  
This command ensures that your ROS nodes know where to find the ROS Master for communication.

If you've modified the default URI, please execute the `start-bridge.sh` script with the new URI address to ensure proper configuration and communication.

For example, if you've changed the default URI to `127.0.0.1`, you should make the following modification to the `compose.yaml`:

```bash
command: ./start-bridge.sh http://127.0.0.1:11311
```

## ROS2 can't receive the topic

The latest releases of Fast-DDS come with the SharedMemory transport enabled by default.  
Therefore, you need to mount shared memory, also known as `/dev/shm`, into every container you intend to communicate with when using Fast-DDS. This ensures proper communication between containers.  
Ensure that you use the same UID as `ros1-bridge` to avoid Fast-DDS shared memory permission issues.

Reference: https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
