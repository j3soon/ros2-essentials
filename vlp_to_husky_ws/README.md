# vlp_to_husky_ws

## Introduction

This repo is used to setup the basic pipeline from the VLP-16 to the Husky robot. The pipeline includes the following steps:
1. VLP-16 data collection in the `vlplidar_ws` workspace.
2. Generate command velocity by the VLP-16 data by the `dummy_controller` in the `dummy_controller_ws` workspace.
3. Send the command velocity to the Husky robot in the `husky_controller_ws` workspace.

## Docker-compose architecture and usage

Since we are going to launch different container at the same time, I think it is better to use `docker-compose` to manage all of the container when
`running in simulation`, `running in real` and `building`. I've separated the `docker-compose` file into three different files for different purpose.

| File name | Usage |
| -------- | -------- |
| `compose-build.yaml` | For building the images, create containers and compile in each workspace. |
| `compose-sim.yaml` | Directly run the simulation world in Gazebo. Since the dummy controller is implemented by Python, it is not required to build again when change the control algorithm. |
| `compose-real.yaml` | Run husky and LiDAR in real world. |

## Test with simulated Husky robot

- Build the images and each workspace by running the following command:
    ```bash
    docker compose -f compose-build.yaml up --build
    ```

- Run the simulation world by running the following command:
    ```bash
    docker compose -f compose-sim.yaml up
    ```

After running the above command, you should be able to see the Husky robot moving in the Gazebo world. You can put some obstacles in the world to
check if husky will stop when it is close to the obstacles. Since the LiDAR data can't collected when the obstacle is too close to the robot (< 1m),
you should put the obstacle at a distance larger than 1m.

If you want to change the control algorithm, you can modify the controlling code `main.py` in the `dummy_controller_ws` workspace and re-run the
command above.

## Test with real Husky robot

- Build the images and each workspace by running the following command:
    ```bash
    docker compose -f compose-build.yaml up --build
    ```

- Run the real world by running the following command:
    ```bash
    docker compose -f compose-real.yaml up
    ```

- Setup husky controller (in container `ros2-husky-ws`)
    ```bash
    cd /home/ros2-agv-essentials/vlp_to_husky_ws/workspaces/husky_ws
    ./udev_rules/install_udev_rules.sh
    ./script/husky-generate.sh 
    source ~/.bashrc
    ./script/husky-bringup.sh
    ```