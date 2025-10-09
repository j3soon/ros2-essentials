# CUDA Toolkit

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_cuda_toolkit.sh)

CUDA Toolkit 12.6 Deb Install. (Not installed by default.)

To enable CUDA Toolkit, set the `CUDA_TOOLKIT_VERSION` argument to `12.6` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

Make sure your GPU driver version supports the CUDA Toolkit version you want to install. For example, CUDA 12.6 requires Linux driver version >=560.35.05 on x86_64. See [Table 3 in the CUDA Toolkit Release Notes](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html) for more information.

> This is often unnecessary when only Python is used, as `pip install torch` typically installs the appropriate version of the CUDA Toolkit automatically.

Please note that if CUDA Toolkit 12.6 is installed while using Isaac Sim 4.5, will need to apply the workaround in the [Isaac ROS](isaac-ros.md) section to prevent CUDA error.
