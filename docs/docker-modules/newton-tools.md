# Newton Tools

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_newton_tools.sh)

URDF-to-USD and MJCF-to-USD converter tools from Newton Physics.

To enable Newton Tools, set the `NEWTON_TOOLS` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

## Usage

The module installs these converters via pip during Docker image build:

- [`urdf-usd-converter`](https://github.com/newton-physics/urdf-usd-converter)
  ```sh
   urdf_usd_converter /path/to/robot.xml /path/to/usd_robot
   # or
   urdf_usd_converter /path/to/robot.xml /path/to/usd_robot --package robot_package=/path/to/assets
  ```
- [`mujoco-usd-converter`](https://github.com/newton-physics/mujoco-usd-converter)
  ```sh
  mujoco_usd_converter /path/to/robot.xml /path/to/usd_robot
  ```

Click the links above to see their usage guide, or use `--help` flag to see usage instructions after installation.

Please note that both of these tools are in alpha stage and features are still limited. You should use the built-in importer in Isaac Sim/Lab for most cases.
