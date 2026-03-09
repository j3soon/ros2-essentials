# Isaac Sim

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_isaac_sim.sh)

Isaac Sim 5.0.0 Binary Install.

> Last tested on commit [d43ad1c](https://github.com/j3soon/ros2-essentials/commit/d43ad1c11075808d5fbf194c5979ffedec213b5e) by [@j3soon](https://github.com/j3soon).

Depends on:

- Vulkan Configuration
- Username

> Note that CUDA Toolkit is not required for Isaac Sim.

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

> **On Host**:
> 
> Quick test using official Docker image:
> 
> ```sh
> scripts/docker_run_official_isaac_sim.sh
> ```

Always make sure to check the console output for any errors.

## Importing URDF

`File > Import` and select the URDF file.

If your URDF file is in the Xacro format, you may need to convert it to the URDF format first:

```sh
XACRO_FILE=<XACRO_FILE>.xacro
URDF_FILE=<URDF_FILE>.urdf
xacro $XACRO_FILE > $URDF_FILE
```

You can also check the URDF hierarchy:

```sh
check_urdf $URDF_FILE
```

> Note: URDF files are often not self-contained and may reference additional resources within their ROS package. For successful import, consider downloading/moving the entire package (such as the `<ROBOT_NAME>_description` directory) along with the URDF file. Otherwise, the import will fail.

## Debugging Physics Simulation

- [Physics Inspector](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/joint_inspector.html) to test the joints and limits.
- [Visualize Colliders](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/physics_static_collision.html#enable-visualization), Joints, and Mass Properties.
- [Simulation Data Visualizer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/ext_isaacsim_inspect_physics.html) to track physics-related data during simulation.
- [Measure Tool](https://docs.omniverse.nvidia.com/extensions/latest/ext_measure-tool.html) that can be installed under `Window > Extensions`.
- `Utilities > Statistics` and select `RTX Scene` for scene optimization metrics.

## Generating OmniGraph for ROS2

Instead of manually creating the OmniGraph via Isaac Sim GUI or Python code, the easier way is to use the [Graph Shortcut tool](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html#graph-shortcut). A concrete example can be found in [PR#83](https://github.com/j3soon/ros2-essentials/pull/83).

## Known Issues

See [official known issues](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/known_issues.html) for Isaac Sim.

```sh
2025-07-10 21:42:18 [11,579ms] [Error] [omni.ext._impl._internal] Failed to import python module isaacsim.core.simulation_manager from /isaac-sim/exts/isaacsim.core.simulation_manager. Error: numpy.dtype size changed, may indicate binary incompatibility. Expected 96 from C header, got 88 from PyObject. Traceback:
Traceback (most recent call last):

A NumPy version >=1.19.5 and <1.27.0 is required for this version of SciPy (detected version 2.2.6)
```

See [Issue #86](https://github.com/j3soon/ros2-essentials/issues/86) for more details.
