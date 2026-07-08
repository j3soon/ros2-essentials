# Isaac Lab

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_isaac_lab.sh)

Isaac Lab git install. The default is `3.0.0-beta2.patch1`; supported `ISAAC_LAB_VERSION` values are `2.3.0`, `2.3.2`, `3.0.0-beta2.patch1`, and `develop`.

> See [Last tested](../last-tested.md) for the latest validation status.

Depends on:

- Vulkan Configuration
- Username
- Isaac Sim

> Note that CUDA Toolkit is not required for Isaac Lab.

Source build from `compose.yaml`:

```yaml
build:
  args:
    ISAAC_LAB_VERSION: "develop"
```

`develop` clones `https://github.com/isaac-sim/IsaacLab.git` at `develop` into `~/IsaacLab`, links the installed Isaac Sim runtime via `_isaac_sim`, then runs `./isaaclab.sh --install`.

[Quick test](https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html#running-pre-built-isaac-lab-container):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/tutorials/00_sim/log_time.py --headless
# View the logs and press Ctrl+C to stop
# tail -f ~/IsaacLab/logs/docker_tutorial/log.txt
```

[Deformable object tutorial](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_deformable_object.html):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/tutorials/01_assets/run_deformable_object.py --viz kit
```

Use `--viz kit` for the Kit visualizer. On first launch, let the Kit window
finish rendering the scene before judging the viewport; the setup logs may
finish before the first visible frame appears.

[Train Cartpole](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task=Isaac-Cartpole-v0 --headless
# or
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Cartpole-v0 --headless
# or
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py --task=Isaac-Cartpole-v0 --headless
```

> **On Host**:
> 
> Quick test using official Docker image:
> 
> ```sh
> scripts/docker_run_official_isaac_lab.sh
> ```

## Known Issues

See [official known issues](https://isaac-sim.github.io/IsaacLab/main/source/refs/issues.html) for Isaac Lab.
