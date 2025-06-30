# Unitree H1

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/h1_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-h1-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-h1-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=h1_ws)](https://github.com/j3soon/ros2-essentials/commits/main/h1_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--h1--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-h1-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-h1-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-h1-ws)

> Please note that this workspace is only tested in simulation.

## Run the container

```bash
# Run the container
cd /home/ros2-essentials/h1_ws/docker
docker compose up -d --build
docker exec -it ros2-h1-ws bash
```

## Testing

### Isaac Sim Examples

[Deploying Policies](https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html#unitree-h1-humanoid-example):

```sh
cd ~/isaacsim
./isaac-sim.sh
```

`Window > Examples > Robotics Examples`, in the `Robotics Examples` window, click `POLICY > Humanoid > LOAD`.

### Isaac Lab Examples

[Training](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html) [environments](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html#comprehensive-list-of-environments) (`Isaac-Velocity-Flat-H1-v0`, `Isaac-Velocity-Rough-H1-v0`):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-H1-v0 --headless
# or
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py --task Isaac-Velocity-Rough-H1-v0 --headless
```

Run [pre-trained model inference](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-H1-v0 --num_envs 32 --use_pretrained_checkpoint
```

Run [H1 Locomotion Showroom Demo](https://isaac-sim.github.io/IsaacLab/main/source/overview/showroom.html):

```sh
cd ~/IsaacLab
export PYTHONPATH=""
./isaaclab.sh -p scripts/demos/h1_locomotion.py
```

> Clearing the PYTHONPATH is necessary to avoid the error:
> 
> ```
> [INFO] Using python from: /home/user/IsaacLab/_isaac_sim/python.sh
> Traceback (most recent call last):
>   File "/home/user/IsaacLab/scripts/demos/h1_locomotion.py", line 23, in <module>
>     import scripts.reinforcement_learning.rsl_rl.cli_args as cli_args  # isort: skip
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/scripts/__init__.py", line 15, in <module>
>     from .gazebo_ros_paths import GazeboRosPaths
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/scripts/gazebo_ros_paths.py", line 32, in <module>
>     from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
> ModuleNotFoundError: No module named 'catkin_pkg'
> There was an error running python
> ```
