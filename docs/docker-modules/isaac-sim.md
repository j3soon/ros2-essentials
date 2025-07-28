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
