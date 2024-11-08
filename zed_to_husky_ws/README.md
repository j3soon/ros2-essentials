# zed_to_husky_ws

This repo is utilized to demonstrate a straightforward end-to-end control flow with using ZED and Clearpath Husky.

## 🌱 Structure 🌱

```
zed_to_husky_ws
├── workspaces
|   ├── dummy_controller_ws
|   ├── husky_ws
|   └── zed_ws
├── .gitignore
└── README.md
```

Since we aim to reuse the code, the repository has a different structure. It includes a folder named `workspaces`, which contains three workspaces: `husky_ws`, `zed_ws`, and `dummy_controller_ws`. The first two workspaces are almost identical to the original, with only a few differences. You can replace both of them with the latest version if you won't be using Gazebo for simulation. However, if Gazebo simulation is necessary, you may need to redo some tasks, such as reintegrating the ZED URDF into the Husky workspace. For guidance, you can refer to the git commit history.

## 🚩 Testing 🚩

The dummy controller offers two modes: 

1. Utilizing both the left and right images from ZED to control the Husky based on the comparison of brightness levels.
2. Utilizing depth estimation from ZED and implementing a P controller to regulate the Husky's movement.

For further details, please refer to the file `dummy_controller_ws/src/dummy_controller/scripts/main.py`.

> Before proceeding to the next step, please ensure that all workspaces are built. This entails opening the workspace with Docker and utilizing `colcon build` to compile the workspace.

### Simulate in Gazebo

```bash=
# In dummy_controller_ws
ros2 run dummy_controller main.py

# In husky_ws
ros2 launch husky_gazebo gazebo.launch.py
```

> Since we've successfully reintegrated ZED into the Husky URDF, there are no additional commands that need to be executed in the `zed_ws` workspace.

After executing the command above, you can add a cube inside Gazebo and move it back and forth in front of the Husky. The dummy controller will then use the observation to control the Husky's behavior.

### Control Real Robot

```bash=
# In dummy_controller_ws
ros2 run dummy_controller main.py

# In husky_ws
cd /home/ros2-agv-essentials/husky_ws
./udev_rules/install_udev_rules.sh
./script/husky-generate.sh 
source ~/.bashrc
./script/husky-bringup.sh

# In zed_ws
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

> For the setup of the Husky and ZED, please refer to the README file in each respective repository for more detailed instructions.
