# Isaac Sim

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_isaac_sim.sh)

Isaac Sim 4.5.0 Binary Install.

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

## Known Issues

```sh
2025-07-10 21:42:18 [11,579ms] [Error] [omni.ext._impl._internal] Failed to import python module isaacsim.core.simulation_manager from /isaac-sim/exts/isaacsim.core.simulation_manager. Error: numpy.dtype size changed, may indicate binary incompatibility. Expected 96 from C header, got 88 from PyObject. Traceback:
Traceback (most recent call last):

A NumPy version >=1.19.5 and <1.27.0 is required for this version of SciPy (detected version 2.2.6)
```

See [Issue #86](https://github.com/j3soon/ros2-essentials/issues/86) for more details.
