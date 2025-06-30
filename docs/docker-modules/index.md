# Docker Modules

## Robot Operating System 2 (ROS 2)

ROS 2 Humble Apt Install.

## CUDA Toolkit

CUDA Toolkit 12.6 Deb Install.

> This is often unnecessary when only Python is used, as `pip install torch` typically installs the appropriate version of the CUDA Toolkit automatically.

## Isaac Sim

Isaac Sim 4.5.0 Binary Install.

Depends on:

- Vulkan Configuration
- Username

> Note that CUDA Toolkit is not required for Isaac Sim.

### On Host

Quick test using official Docker image:

```sh
scripts/docker_run_official_isaac_sim.sh
```

### In Container

Compatibility test:

```sh
cd ~/isaac-sim-comp-check
./omni.isaac.sim.compatibility_check.sh
```

Quick test:

```sh
cd ~/isaacsim
./python.sh standalone_examples/api/isaacsim.core.api/time_stepping.py
# or
./python.sh standalone_examples/api/isaacsim.core.api/simulation_callbacks.py
```

Launch GUI:

```sh
cd ~/isaacsim
./isaac-sim.sh
```

## Isaac Lab

Isaac Lab 2.1.0 Git Install.

Depends on:

- Vulkan Configuration
- Username
- Isaac Sim

> Note that CUDA Toolkit is not required for Isaac Lab.

### On Host

Quick test using official Docker image:

```sh
scripts/docker_run_official_isaac_lab.sh
```

### In Container

[Quick test](https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html#running-pre-built-isaac-lab-container):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/tutorials/00_sim/log_time.py --headless
# View the logs and press Ctrl+C to stop
```

[Train Cartpole](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task=Isaac-Cartpole-v0 --headless
# or
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Cartpole-v0 --headless
# or
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py --task=Isaac-Cartpole-v0 --headless
```

## Isaac ROS

> Note: This is a work in progress. Currently only the base Isaac ROS is installed. Running examples requires additional dependencies that are not yet included in this setup.

Isaac ROS 3.2.

Depends on:

- ROS 2 Humble
- CUDA Toolkit
- and more...
