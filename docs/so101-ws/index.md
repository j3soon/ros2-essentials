# LeRobot SO-ARM 101 Pro

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/tree/main/so101_ws)
[![build](https://img.shields.io/github/actions/workflow/status/j3soon/ros2-essentials/build-so101-ws.yaml?label=build)](https://github.com/j3soon/ros2-essentials/actions/workflows/build-so101-ws.yaml)
[![GitHub last commit](https://img.shields.io/github/last-commit/j3soon/ros2-essentials?path=so101_ws)](https://github.com/j3soon/ros2-essentials/commits/main/so101_ws)

[![DockerHub image](https://img.shields.io/badge/dockerhub-j3soon/ros2--so101--ws-important.svg?logo=docker)](https://hub.docker.com/r/j3soon/ros2-so101-ws/tags)
![Docker image arch](https://img.shields.io/badge/arch-amd64-blueviolet)
![Docker image version](https://img.shields.io/docker/v/j3soon/ros2-so101-ws)
![Docker image size](https://img.shields.io/docker/image-size/j3soon/ros2-so101-ws)

## 🐳 Start Container

> Make sure your system meets the [system requirements](https://j3soon.github.io/ros2-essentials/#system-requirements) and have followed the [setup instructions](https://j3soon.github.io/ros2-essentials/#setup) before using this workspace.

Run the following commands in a Ubuntu desktop environment. If you are using a remote server, make sure you're using a terminal within a remote desktop session (e.g., VNC) instead of SSH (i.e., don't use `ssh -X` or `ssh -Y`).

```sh
cd ~/ros2-essentials/so101_ws/docker
docker compose build
xhost +local:docker
docker compose up -d
# The initial build will take a while, please wait patiently.
```

> If your user's UID is `1000`, you may replace the `docker compose build` command with `docker compose pull`.

The commands in the following sections assume that you are inside the Docker container:

```sh
# in a new terminal
docker exec -it ros2-so101-ws bash
```

If the initial build somehow failed, run:

```sh
rm -r build install
colcon build --symlink-install
```

Once you have finished testing, you can stop and remove the container with:

```sh
docker compose down
```

> If you encountered unexpected issues when using Isaac Sim, see [Known Issues](../docker-modules/isaac-sim.md#known-issues) for more details.

## Hardware Assembly

> Skip this section if you have a pre-assembled SO-101 arm.

The two main documents we'll follow here are [the official Hugging Face SO-101 docs](https://huggingface.co/docs/lerobot/so101) and that [provided by seeedstudio](https://wiki.seeedstudio.com/lerobot_so100m_new/).

We take the SO-ARM101 Arm Pro Motor Kit + 3D Printed Parts by seeedstudio as an example. All dependencies are pre-installed in the Docker container for all setups. Note that working on the Ubuntu host or using Windows 11 to configure the motors is not needed for this workspace.

### Configure the Motors

STS3215 Servos (STS-3215-C0XX)

- C001 x1 (1/345)
- C044 x2 (1/191)
- C046 x3 (1/147)
- C047 x6 (1/345)

!!! danger

    Always double check the voltage of the power supply and the motor. The 5V power supply is for the leader arm (7.4V motors), and the 12V power supply is for the follower arm (12V motors). Mixing them up will burn your motors.

Follow [the seeedstudio guide](https://wiki.seeedstudio.com/lerobot_so100m_new/#configure-the-motors) to configure the motors. Specifically, you'll use the commands below.

Use

```sh
lerobot-find-port
```

to test `/dev/ttyACM*` ports.

- Leader Arm

  5V Power Supply (AD0301-0503500F)
  
  | Leader-Arm Axis     | Motor | Gear Ratio | Code          |
  | ------------------- | :---: | :--------: | :-----------: |
  | Base / Shoulder Pan |   1   |  1 / 191   | STS-3215-C044 |
  | Shoulder Lift       |   2   |  1 / 345   | STS-3215-C001 |
  | Elbow Flex          |   3   |  1 / 191   | STS-3215-C044 |
  | Wrist Flex          |   4   |  1 / 147   | STS-3215-C046 |
  | Wrist Roll          |   5   |  1 / 147   | STS-3215-C046 |
  | Gripper             |   6   |  1 / 147   | STS-3215-C046 |

- Follower Arm

  12V Power Supply (AD0301-1202500F) (We use the high torque **PRO** version here)
  
  | Follower-Arm Axis   | Motor | Gear Ratio | Code          |
  | ------------------- | :---: | :--------: | :-----------: |
  | Base / Shoulder Pan |   1   |  1 / 345   | STS-3215-C047 |
  | Shoulder Lift       |   2   |  1 / 345   | STS-3215-C047 |
  | Elbow Flex          |   3   |  1 / 345   | STS-3215-C047 |
  | Wrist Flex          |   4   |  1 / 345   | STS-3215-C047 |
  | Wrist Roll          |   5   |  1 / 345   | STS-3215-C047 |
  | Gripper             |   6   |  1 / 345   | STS-3215-C047 |

To configure the motors for the leader arm:

```
# 5V
sudo chmod 666 /dev/ttyACM*
lerobot-setup-motors \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1  # <- paste here the port found at previous step
```

To configure the motors for the follower arm:

```
# 12V
sudo chmod 666 /dev/ttyACM*
lerobot-setup-motors \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0  # <- paste here the port found at previous step
```

!!! note

    The two connector ports on each motor has identical functionalities, so you can connect to either port while configuring the motors.

    Use a [flat screwdriver](https://youtu.be/9Pl67-vVYhw) to quickly install/remove connectors to/from the motors.

    Always plug the follower arm to the computer first, and plug the leader arm later for the correct tty order used in this doc.

### Assemble the Arm

After configuring the motors, you can now assemble the arm following the [official guide](https://huggingface.co/docs/lerobot/so101?example=Linux&setup_motors=Command#clean-parts). There are also [many videos](https://youtu.be/-8uQOWK-IGo) online you can refer to.

!!! note

    If some holes seems a little bit smaller than screws, try to rotate the screw with a drive manually, the plastic is soft enough to be grind away.

    If some parts doesn't seem to be able to fit in another, double-check if the orientation is off by 180 degrees.

    You can assemble the entire arm and then connect the connectors to the motors after.

    It's normal to spend a few hours assembling the arm for the first time. If you have questions, don't hesitate to ask the maintainers of this workspace.

### Calibration

After assembling the arm, you can now calibrate the arm following the [official guide](https://huggingface.co/docs/lerobot/so101?example=Linux&setup_motors=Command#calibrate). Specifically, you'll use the commands below.

```sh
# 12V
sudo chmod 666 /dev/ttyACM*
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm
```

```sh
#5V
sudo chmod 666 /dev/ttyACM*
lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=my_awesome_leader_arm
```

> If you encountered the following error:
>
> ```
> ValueError: Magnitude 2114 exceeds 2047 (max for sign_bit_index=11)
> ```
>
> Try [disconnect the arm's power supply](https://github.com/huggingface/lerobot/issues/1296#issuecomment-2994779859) and reconnect it. (i.e., reboot trick for the arms)

### Teleoperation

After calibrating the arm, you can now teleoperate the arm following the [official guide](https://huggingface.co/docs/lerobot/il_robots#teleoperate). Specifically, you'll use the commands below.

```sh
sudo chmod 666 /dev/ttyACM*
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=my_awesome_leader_arm
```

## References

- [Official Hugging Face SO-101 Documentation](https://huggingface.co/docs/lerobot/so101)
- [Seeed Studio Wiki for LeRobot SO-ARM101](https://wiki.seeedstudio.com/lerobot_so100m_new/)
- [Assemble SO-ARM101 (Leader) – Part 1 (YouTube)](https://youtu.be/-8uQOWK-IGo)
- [Assemble and Calibrate SO-100: LeRobot Tutorial #7 by Jess Moss (YouTube)](https://youtu.be/FioA2oeFZ5I)
