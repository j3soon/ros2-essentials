# deeplab_ws

This workspace will set up a Deeplab v3+ model in ROS 2 Humble.  
> The model was modified from [VainF/DeepLabV3Plus-Pytorch](https://github.com/VainF/DeepLabV3Plus-Pytorch)

## 🌱 Structure 🌱

Here is the structure of this repository:

```
deeplab_ws
├── .devcontainer
|   ├── devcontainer.json
|   └── postCreateCommand.sh
├── docker
|   ├── cache
|   |   └── .gazebo
|   |   |   └── .gitkeep
|   |   └── torch
|   |   |   └── .gitkeep
|   ├── .bashrc
|   ├── Dockerfile
|   └── docker-compose.yaml
├── install
├── build
├── log
├── src
|   └── deeplab
|   |   ├── deeplab
|   |   |   ├── network
|   |   |   └── weights
|   |   ├── launch
|   |   ├── scripts
|   |   ├── CMakeLists.txt 
|   |   └── package.xml
└── README.md
```

> ```build``` / ```install``` / ```log``` folders will appear once you've built the packages.

## 🚩 Testing 🚩

Run the command below in the terminal:

```bash
cd $ROS2_WS
colcon build
ros2 launch deeplab deeplab_launch.py
```

> The default input/output topics are `/realsense/image_raw` and `/deeplab/result`.  
> You can modify the default topics in the `deeplab_ws/src/deeplab/launch/deeplab_launch.py`.  
> Additionally, you can directly pass the arguments through the launch command:  
> ```bash
> ros2 launch deeplab deeplab_launch.py \
>   input_image_topic:=/custom/input \
>   output_image_topic:=/custom/output
> ```
