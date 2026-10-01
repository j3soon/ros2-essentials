# Isaac Lab

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_isaac_lab.sh)

Isaac Lab git install. The default is `3.0.0-EA`. Supported `ISAAC_LAB_VERSION` values are `2.3.0`, `2.3.2`, `3.0.0-beta2.patch1`, `3.0.0-EA`, and `develop`.

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

[Quick test](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/setup/installation/index.html):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/tutorials/00_sim/log_time.py
# View the logs and press Ctrl+C to stop
# tail -f ~/IsaacLab/logs/docker_tutorial/log.txt
```

[Deformable object tutorial](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/tutorials/01_assets/run_deformable_object.html):

```sh
cd ~/IsaacLab
./isaaclab.sh -p scripts/tutorials/01_assets/run_deformable_object.py --viz kit
```

Use `--viz kit` for the Kit visualizer. On first launch, let the Kit window
finish rendering the scene before judging the viewport. The first run may
spend several minutes compiling shaders even after the startup log appears.
If the desktop offers to close an unresponsive Kit window during that time,
choose Wait while the process is still active. Later launches use the caches.
Isaac Lab 3.0.0-EA also warns that `isaaclab.sh` will be replaced by
`uv run isaaclab` in a future release.

[Train Cartpole](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/how-to/run_rl_training.html):

```sh
cd ~/IsaacLab
./isaaclab.sh train --rl_library rl_games --task Isaac-Cartpole
# or
./isaaclab.sh train --rl_library rsl_rl --task Isaac-Cartpole
# or
./isaaclab.sh train --rl_library skrl --task Isaac-Cartpole
```

## Known Issues

See [official known issues](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/refs/issues.html) for Isaac Lab.
