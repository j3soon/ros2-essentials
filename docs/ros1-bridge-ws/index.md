# ROS1 Bridge

This workspace is utilized to create a bridge between ROS1 and ROS2-humble.

# ◻️ Introduction ◻️

`ros1_bridge` provides a network bridge that enables the exchange of messages between ROS 1 and ROS 2.  
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
|   ├── .dockerignore
|   ├── .env
|   ├── compose.yaml
|   ├── compose.debug.yaml
|   ├── Dockerfile
|   └── start-bridge.sh
└── README.md
```

# 🚩 How to use 🚩

> The docker compose includes two services: `ros1-bridge` and `ros1-bridge-build`. The `ros1-bridge` service is typically sufficient for regular use, while `ros1-bridge-build` is intended for debugging, as it retains all the necessary build tools in the docker image.
>
> If you are not debugging `ros1-bridge`, it is recommended to use the terminal rather than VScode-devcontainer.  
> By default, the VScode-devcontainer uses the `ros1-bridge-build` service.

## 1. Build the docker image

While building the image directly from the Dockerfile is possible, it may not be the most efficient choice. To save time, you can pull the image from Dockerhub instead of compiling it from the source.

If you still prefer to build the image yourself, please follow the instructions below:

- VScode user
  - Open the workspace in VScode, press `F1`, and enter `> Dev Containers: Rebuild Container`.
- Terminal user
  - Open a terminal, change the directory to the docker folder, and type `docker compose build`.

> Please note that the build process may take approximately 1 hour to complete, with potential delays depending on your computer's performance and network conditions.

## 2. Adjust the parameters in the `.env` file

We've placed all ROS-related parameters in the `.env` file. Please adjust these parameters according to your environment. By default, we set the `ROS1` master at `127.0.0.1`, and the `ROS2` domain ID to `0`. These settings should work for most scenarios. 

Please note that if these parameters are not configured correctly, the `ros1_bridge` will not function properly!

## 3. Start the container

- VScode user
  - If you build the image through the devcontainer, it will automatically start the container.  
    After you get into the container, type `./start-bridge.sh` in the terminal.
- Terminal user
  - Open a terminal, change the directory to the docker folder, and type `docker compose up`.

## 4. Launch rosmaster in ROS1

> This step is automatically executed when you run `docker compose up`.

As mentioned in https://github.com/ros2/ros1_bridge/issues/391, you should avoid using `roscore` in ROS1 to prevent the issue of not bridging `/rosout`.  
Instead, use `rosmaster --core` as an alternative.

## 5. Begin communication

You have successfully executed all the instructions.  
Now, you can proceed to initiate communication between ROS1 and ROS2-humble.

Please keep in mind that the bridge will be established only when there are matching publisher-subscriber pairs active for a topic on either side of the bridge.

# ✨ Example ✨

## Run the bridge and the example talker and listener

Before beginning the example, ensure you have four containers ready:

- `ros-core`
- `ros2-ros1-bridge-ws`
- `ros1`
- `ros2`

> When using `ros1-bridge` in your application scenarios, you only need the `ros-core` and `ros2-ros1-bridge-ws` containers. Please replace the `ros1` and `ros2` containers with your application containers, as those are only included for demonstration purposes and are not required for using `ros1-bridge`.  
> 
> Furthermore, ensure that you mount `/dev/shm` into both the `ros2-ros1-bridge-ws` and `ros2` containers, and that all containers share the host network.

### 1. Start the `ros1_bridge` and other container

```bash
# In docker folder
docker compose up
```

This command will start the four containers mentioned above.

### 2. Run the talker and listener node

We run the listener node in the `ros1` container and the talker node in the `ros2` container. You can run the talker node in `ros1` and the listener node in `ros2` if you'd like. To achieve this, modify the command provided below.

#### ROS1

```bash
docker exec -it ros1 /ros_entrypoint.sh bash
# Inside ros1 container
rosrun roscpp_tutorials listener
# or
# rosrun roscpp_tutorials talker
```

#### ROS2

```bash
docker exec -it ros2 /ros_entrypoint.sh bash
# Inside ros2 container
# Use the same UID as ros1_bridge to prevent Fast-DDS shared memory permission issues.
# Ref: https://github.com/j3soon/ros2-essentials/pull/9#issuecomment-1795743063
useradd -ms /bin/bash user
su user
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
# or
# ros2 run demo_nodes_cpp listener
```

Certainly, you can try the [example](https://github.com/ros2/ros1_bridge#example-1-run-the-bridge-and-the-example-talker-and-listener) provided by `ros1_bridge`. However, there's no need to source the setup script within the `ros2-ros1-bridge-ws` container, simply starting the container will suffice.

# 🔍 Troubleshooting 🔍

> If you are trying to debug `ros1_bridge`, it is recommended to use the `ros1-bridge-build` service in docker compose. It contains all the necessary build tools, which should be helpful for you.

## Failed to contact ros master

Before launching `ros-core`, make sure to adjust the `ROS_MASTER_URI` correctly. For more information, please check the `.env` file and [this section](#2-adjust-the-parameters-in-the-env-file).

You can replace `127.0.0.1` with the actual IP address or hostname of your ros master. This configuration ensures that your ros nodes know where to find the ros master for communication. Remember, in addition to modifying the parameters for `ros1_bridge`, you also need to adjust the parameters for your own container!

## ROS2 can't receive the topic

The latest releases of Fast-DDS come with the shared memory transport enabled by default. Therefore, you need to mount shared memory, also known as `/dev/shm`, into every container you intend to communicate with when using Fast-DDS. This ensures proper communication between containers. Ensure that you use the same UID as `ros2-ros1-bridge-ws` to avoid Fast-DDS shared memory permission issues.

Reference: https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
