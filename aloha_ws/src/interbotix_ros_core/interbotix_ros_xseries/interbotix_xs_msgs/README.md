# Interbotix X-Series Messages ROS Package

## Overview
This package builds the common messages, services, and actions used by the Interbotix X-Series platforms.

# Structure
The *interbotix_xs_msgs* package contains no nodes, rather it defines a common location to store message, service, and action definitions used by X-Series platforms.

# Usage
Simply build this package and include any generated message in your own custom node or script. For example, if you wanted to control an X-Series manipulator using a joystick, you would include the ArmJoy message in your program's header.
- C++
```C++
#include "interbotix_xs_msgs/ArmJoy.h"
```
- Python
```python
from interbotix_xs_msgs.msg import ArmJoy
```

The table below provides a list and use case for each type defined in this package.

| Type                          | Description                                                                        | Use Case                 |
| :---------------------------: | :--------------------------------------------------------------------------------- | :----------------------- |
| JointSingleCommand.msg        | Command a desired joint                                                            | Common                   |
| JointGroupCommand.msg         | Command the joints in the specified joint group.                                   | Common                   |
| JointTrajectoryCommand.msg    | Command a joint trajectory to the desired joint(s).                                | Common                   |
| JointTemps.msg                | Holds the temperatures [C] for the specified joints                                | Common                   |
| ArmJoy.msg                    | Maps raw 'joy' commands to more specific ones to control an Interbotix manipulator | Joystick Arm Control     |
| LocobotJoy.msg                | Maps raw 'joy' commands to more specific ones to control an Interbotix LoCoBot     | Joystick LoCoBot Control |
| HexJoy.msg                    | Maps raw 'joy' commands to more specific ones to control an Interbotix hexapod     | Joystick Hexapod Control |
| TurretJoy.msg                 | Maps raw 'joy' commands to more specific ones to control an Interbotix turret      | Joystick Turret Control  |
| MotorGains.srv                | Set PID gains                                                                      | Common                   |
| OperatingModes.srv            | Set Operating Modes                                                                | Common                   |
| Reboot.srv                    | Reboot motors                                                                      | Common                   |
| RegisterValues.srv            | Set or get the register(s) value(s) from motor(s)                                  | Common                   |
| RobotInfo.srv                 | Get robot information                                                              | Common                   |
| TorqueEnable.srv              | Torque joints on/off                                                               | Common                   |
