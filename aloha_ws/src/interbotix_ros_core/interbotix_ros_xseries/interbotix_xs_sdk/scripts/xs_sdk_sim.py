#!/usr/bin/env python3

# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from array import ArrayType
import math
import sys
import threading
from typing import Dict, List, Union

from interbotix_xs_msgs.msg import (
    JointGroupCommand,
    JointSingleCommand,
    JointTrajectoryCommand,
)
from interbotix_xs_msgs.srv import (
    MotorGains,
    OperatingModes,
    Reboot,
    RegisterValues,
    RobotInfo,
    TorqueEnable
)
import numpy as np
import rclpy
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
import yaml

LOGGER = get_logger('interbotix_xs_sdk.xs_sdk_sim')


class InterbotixRobotXS(Node):
    """
    Class that simulates the actual InterbotixRobotXS class.

    :details: allows to use the same higher level ROS packages or Python API code on a
        simulated robot with no change to the code
    """

    rd: URDF = None
    """Holds the URDF loaded from the ROS parameter server"""

    timer_hz: float = 30.0
    """Rate [Hz] at which the ROS Timer (in charge of publishing joint states) should run"""

    js_topic: str = None
    """Joint States topic"""

    move_thresh: int = 300
    """Threshold [ms] above which desired goal positions should be split up into waypoints and
    simulated"""

    execute_joint_traj: bool = False
    """`True` if a trajectory is currently being executed; `False` otherwise"""

    joint_states = JointState()
    """Current state of all joints"""

    cmd_mutex = threading.Lock()
    """Mutex to prevent reading/writing to self.commands from different threads"""

    commands: Dict[str, float] = {}
    """Holds the desired commands for each joint"""

    sleep_map: Dict[str, float] = {}
    """Maps a joint name with its sleep position [rad]"""

    group_map: Dict[str, Dict] = {}
    """Maps a group name with a JointGroup-type dictionary (refer to the top of the xs_sdk_obj.hpp
    file for reference)"""

    motor_map: Dict[str, Dict] = {}
    """Maps a joint name with a MotorState-type dictionary (refer to the top of the #
    xs_sdk_obj.hpp file for reference)"""

    gripper_map: Dict[str, Dict] = {}
    """Maps a gripper name with a Gripper-type dictionary (refer to the top of the xs_sdk_obj.hpp
    file for reference)"""

    js_index_map: Dict[str, int] = {}
    """Maps a joint name with its index in self.joint_states.position"""

    gripper_order: List[str] = []
    """List of grippers as they appear in the 'joint_order' parameter in the Motor Configs YAML
    file"""

    def __init__(self):
        """Construct the InterbotixRobotXS simulaiton node."""
        super().__init__('xs_sdk_sim')

        self.declare_parameter('motor_configs', '')
        self.declare_parameter('mode_configs', '')
        self.declare_parameter('robot_description', '')

        self.get_urdf_info()

        if not self.robot_get_motor_configs():
            sys.exit(1)

        self.create_service(
            TorqueEnable,
            'torque_enable',
            self.robot_srv_torque_enable)
        self.create_service(
            Reboot,
            'reboot_motors',
            self.robot_srv_reboot_motors)
        self.create_service(
            RobotInfo,
            'get_robot_info',
            self.robot_srv_get_robot_info)
        self.create_service(
            OperatingModes,
            'set_operating_modes',
            self.robot_srv_set_operating_modes)
        self.create_service(
            MotorGains,
            'set_motor_pid_gains',
            self.robot_srv_set_motor_pid_gains)
        self.create_service(
            RegisterValues,
            'set_motor_registers',
            self.robot_srv_set_motor_registers)
        self.create_service(
            RegisterValues,
            'get_motor_registers',
            self.robot_srv_get_motor_registers)

        self.create_subscription(
            JointGroupCommand,
            'commands/joint_group',
            self.robot_sub_command_group,
            10)
        self.create_subscription(
            JointSingleCommand,
            'commands/joint_single',
            self.robot_sub_command_single,
            10)
        self.create_subscription(
            JointTrajectoryCommand,
            'commands/joint_trajectory',
            self.robot_sub_command_traj,
            10)

        self.pub_joint_states = self.create_publisher(JointState, self.js_topic, 10)
        self.create_timer(1.0/self.timer_hz, self.robot_update_joint_states)

        LOGGER.info("Interbotix 'xs_sdk_sim' node is up!")

    def get_urdf_info(self) -> None:
        """Load the URDF to get joint limit information."""
        if self.has_parameter('robot_description'):
            self.rd = URDF.from_xml_string(
                self.get_parameter('robot_description').get_parameter_value().string_value)

    def robot_get_motor_configs(self) -> bool:
        """
        Load the 'motor_configs' and 'mode_configs' YAML files and populate class dictionaries.

        :return: `True` if configs were retrieved, `False` otherwise
        """
        # get filepaths to the YAML files from the ROS parameter server
        motor_configs_filepath = self.get_parameter(
            'motor_configs').get_parameter_value().string_value
        mode_configs_filepath = self.get_parameter(
            'mode_configs').get_parameter_value().string_value

        # try to open and load both files into local variables
        try:
            with open(motor_configs_filepath, 'r') as yamlfile:
                motor_configs = yaml.safe_load(yamlfile)
        except IOError:
            LOGGER.error('Motor Config File was not found.')
            return False
        LOGGER.info(
            f'Loaded motor configs from `{motor_configs_filepath}`.'
        )

        try:
            with open(mode_configs_filepath, 'r') as yamlfile:
                mode_configs = yaml.safe_load(yamlfile)
        except IOError:
            mode_configs = {}
            LOGGER.warning('Mode Config file was not found. Using defaults.')
        LOGGER.info(f'Loaded mode configs from `{mode_configs_filepath}`.')

        joint_order = motor_configs['joint_order']
        sleep_positions = motor_configs['sleep_positions']

        self.js_topic = motor_configs['joint_state_publisher']['topic_name']

        motor_groups = motor_configs.get('groups', {})
        grippers = motor_configs.get('grippers', {})
        motors = motor_configs['motors']

        mode_groups = mode_configs.get('groups', {})
        singles = mode_configs.get('singles', {})

        # populate self.gripper_map
        if grippers:
            for gpr, items in grippers.items():
                self.gripper_map[gpr] = {
                    'horn_radius': items['horn_radius'],
                    'arm_length': items['arm_length'],
                    'left_finger': items['left_finger'],
                    'right_finger': items['right_finger']
                }
        else:
            self.gripper_map = {}

        # populate self.sleep_map, self.gripper_order, and self.js_index_map
        # also, initialize self.joint_states
        for indx in range(len(joint_order)):
            self.sleep_map[joint_order[indx]] = sleep_positions[indx]
            if joint_order[indx] in self.gripper_map:
                self.gripper_order.append(joint_order[indx])
                self.gripper_map[joint_order[indx]]['js_index'] = indx
            self.js_index_map[joint_order[indx]] = indx
            self.joint_states.name.append(joint_order[indx])
            self.joint_states.position.append(sleep_positions[indx])

        # continue to populate self.joint_states with gripper finger info
        for gpr in self.gripper_order:
            fingers = ['left_finger', 'right_finger']
            lin_dist = self.robot_convert_angular_position_to_linear(gpr, self.sleep_map[gpr])
            for finger in fingers:
                fngr = self.gripper_map[gpr][finger]
                self.js_index_map[fngr] = len(self.js_index_map)
                self.joint_states.name.append(fngr)
                self.joint_states.position.append(
                    lin_dist if finger == 'left_finger' else -lin_dist
                )

        # populate self.motor_map
        for name in joint_order:
            self.motor_map[name] = {'motor_id': motors[name]['ID']}

        # populate self.group_map
        groups = list(motor_groups)
        groups.insert(0, 'all')
        motor_groups['all'] = joint_order
        mode_groups['all'] = {}
        for grp in groups:
            joint_names = motor_groups[grp]
            joint_group = {}
            joint_group['joint_names'] = joint_names
            joint_group['joint_num'] = len(joint_names)
            joint_group['joint_ids'] = [self.motor_map[name]['motor_id'] for name in joint_names]
            self.group_map[grp] = joint_group
            mode = mode_groups[grp].get('operating_mode', 'position')
            profile_type = mode_groups[grp].get('profile_type', 'velocity')
            profile_velocity = mode_groups[grp].get('profile_velocity', 0)
            profile_acceleration = mode_groups[grp].get('profile_acceleration', 0)
            self.robot_set_operating_modes(
                'group',
                grp,
                mode,
                profile_type,
                profile_velocity,
                profile_acceleration
            )

        # continue to populate self.motor_map
        if singles:
            for sgl in singles:
                info = singles[sgl]
                mode = info.get('operating_mode', 'position')
                profile_type = info.get('profile_type', 'velocity')
                profile_velocity = info.get('profile_velocity', 0)
                profile_acceleration = info.get('profile_acceleration', 0)
                self.robot_set_joint_operating_mode(
                    sgl,
                    mode,
                    profile_type,
                    profile_velocity,
                    profile_acceleration
                )

        return True

    def robot_set_operating_modes(
        self,
        cmd_type: str,
        name: str,
        mode: str,
        profile_type: str,
        profile_velocity: int,
        profile_acceleration: int
    ) -> None:
        """
        Set the operating mode for a specific group of motors or a single motor.

        :param cmd_type: set to 'group' if changing the operating mode for a group of motors or
            'single' if changing the operating mode for a single motor
        :param name: desired motor group name if cmd_type is set to 'group' or the desired motor
            name if cmd_type is set to 'single'
        :param mode: desired operating mode (either 'position', 'linear_position', 'ext_position',
            'velocity', 'pwm', 'current', or 'current_based_position')
        :param profile_type: set to 'velocity' for a Velocity-based Profile or 'time' for a
            Time-based Profile (only 'time' is supported in the simulator)
        :param profile_velocity: passthrough to the Profile_Velocity register on the motor
        :param profile_acceleration: passthrough to the Profile_Acceleration register on the motor
        """
        if (cmd_type == 'group' and name in self.group_map):
            for joint_name in self.group_map[name]['joint_names']:
                self.robot_set_joint_operating_mode(
                    joint_name,
                    mode,
                    profile_type,
                    profile_velocity,
                    profile_acceleration
                )
            self.group_map[name]['mode'] = mode
            self.group_map[name]['profile_type'] = profile_type
            LOGGER.info((
                f"The operating mode for the '{name}' group was changed to '{mode}'."))
        elif (cmd_type == 'single' and name in self.motor_map):
            self.robot_set_joint_operating_mode(
                name,
                mode,
                profile_type,
                profile_velocity,
                profile_acceleration
            )
            LOGGER.info((
                f"The operating mode for the '{name}' joint was changed to '{mode}'."
            ))
        elif (
            (cmd_type == 'group' and name not in self.group_map) or
            (cmd_type == 'single' and name not in self.motor_map)
        ):
            LOGGER.info((
                f"The '{name}' joint/group does not exist. "
                'Was it added to the motor config file?'
            ))
        else:
            LOGGER.error(
                "Invalid command for argument 'cmd_type' while setting operating mode."
            )

    def robot_set_joint_operating_mode(
        self,
        name: str,
        mode: str,
        profile_type: str,
        profile_velocity: int,
        profile_acceleration: int
    ) -> None:
        """
        Set the operating mode for a single motor.

        :param name: desired motor name
        :param mode: desired operating mode (either 'position', 'linear_position', 'ext_position',
            'velocity', 'pwm', 'current', or 'current_based_position')
        :param profile_type: set to 'velocity' for a Velocity-based Profile or 'time' for a
            Time-based Profile (only 'time' is supported in the simulator)
        :param profile_velocity: passthrough to the Profile_Velocity register on the motor
        :param profile_acceleration: passthrough to the Profile_Acceleration register on the motor
        """
        self.motor_map[name]['mode'] = mode
        self.motor_map[name]['profile_type'] = profile_type
        self.motor_map[name]['profile_velocity'] = profile_velocity
        self.motor_map[name]['profile_acceleration'] = profile_acceleration

    def robot_write_commands(
        self,
        name: str,
        commands: Union[List[float], ArrayType]
    ) -> None:
        """
        Command a desired group of motors with the specified commands.

        :param name: desired motor group name
        :param commands: vector of commands (order matches the order specified in the 'groups'
            section in the motor config file)
        :details: commands are processed differently based on the operating mode specified for the
            motor group
        """
        joints = self.group_map[name]['joint_names']
        for x in range(len(joints)):
            self.robot_write_joint_command(joints[x], commands[x])

    def robot_write_joint_command(
        self,
        name: str,
        command: float
    ) -> None:
        """
        Command a desired motor with the specified command.

        :param name: desired motor name
        :param command: motor command
        :details: the command is processed differently based on the operating mode specified for
            the motor
        """
        motor = self.motor_map[name]
        mode = motor['mode']
        prof_vel = motor['profile_velocity']
        with self.cmd_mutex:
            # create a trajectory of 'waypoints' to execute if motion should take longer than
            # 'self.move_thresh' milliseconds
            if 'position' in mode and prof_vel > self.move_thresh:
                num_itr = int(round(prof_vel / 1000.0 * self.timer_hz + 1))
                self.commands[name] = list(
                    np.linspace(
                        command,
                        self.joint_states.position[self.js_index_map[name]],
                        num_itr
                    )
                )
            else:
                self.commands[name] = command

    def robot_convert_linear_position_to_radian(
        self,
        name: str,
        linear_position: float
    ) -> float:
        """
        Convert a linear distance between two gripper fingers into an angular position.

        :param name: name of the gripper servo to command
        :param linear_position: desired distance [m] between the two gripper fingers
        :return result: angular position [rad] that achieves the desired linear distance
        """
        half_dist = linear_position / 2.0
        arm_length = self.gripper_map[name]['arm_length']
        horn_radius = self.gripper_map[name]['horn_radius']
        return math.pi/2.0 - math.acos(
            (horn_radius**2 + half_dist**2 - arm_length**2) / (2 * horn_radius * half_dist)
        )

    def robot_convert_angular_position_to_linear(
        self,
        name: str,
        angular_position: float
    ) -> float:
        """
        Convert an angular position into the linear distance from gripper horn to finger.

        :param name: name of the gripper sevo to command
        :param angular_position: desired gripper angular position [rad]
        :return: linear position [m] from a gripper finger to the center of the gripper servo horn
        """
        arm_length = self.gripper_map[name]['arm_length']
        horn_radius = self.gripper_map[name]['horn_radius']
        a1 = horn_radius * math.sin(angular_position)
        c = math.sqrt(horn_radius**2 - a1**2)
        a2 = math.sqrt(arm_length**2 - c**2)
        return a1 + a2

    def robot_sub_command_group(self, msg: JointGroupCommand) -> None:
        """
        Command a group of joints from a ROS Subscriber callback.

        :param msg: JointGroupCommand message dictating the joint group to command along with the
            actual commands
        :details: refer to the message definition for details
        """
        self.robot_write_commands(msg.name, msg.cmd)

    def robot_sub_command_single(self, msg: JointSingleCommand) -> None:
        """
        Command a single joint from a ROS subscriber callback.

        :param msg: JointSingleCommand message dictating the joint to command along with the actual
            command
        :details: refer to the message definition for details
        """
        self.robot_write_joint_command(msg.name, msg.cmd)

    def robot_sub_command_traj(self, msg: JointTrajectoryCommand) -> None:
        """
        Command a joint trajectory from a ROS Subscriber callback.

        :param msg: JointTrajectoryCommand message dictating the joint(s) to command along with the
            desired trajectory
        :details: refer to the message definition for details
        """
        # check to make sure there is no trajectory currently running and that it is valid
        if self.execute_joint_traj:
            LOGGER.warning('Trajectory rejected since joints are still moving.')
            return
        if len(msg.traj.points) < 2:
            LOGGER.warning('Trajectory has fewer than 2 points. Aborting...')
            return

        # get the mode and joint_names
        mode = None
        joint_names = []
        if (msg.cmd_type == 'group'):
            mode = self.group_map[msg.name]['mode']
            joint_names = self.group_map[msg.name]['joint_names']
        elif (msg.cmd_type == 'single'):
            mode = self.motor_map[msg.name]['mode']
            joint_names.append(msg.name)

        # check to see if the initial positions match the current joint states
        if len(msg.traj.points[0].positions) == len(joint_names):
            for x in range(len(joint_names)):
                expected_state = msg.traj.points[0].positions[x]
                actual_state = self.joint_states.position[self.js_index_map[joint_names[x]]]
                if not (abs(expected_state - actual_state) < 0.01):
                    LOGGER.warning(
                        f"The '{joint_names[x]}' joint is not at the correct initial state.")
                    LOGGER.warning((
                        f"Expected state: '{expected_state:.2f}', "
                        f"Actual State: '{actual_state:.2f}'."))

        # execute the trajectory
        self.exectue_joint_traj = True

        points: List[JointTrajectoryPoint] = msg.traj.points
        time_start = self.get_clock().now()
        for x in range(1, len(points)):
            if msg.cmd_type == 'group':
                if 'position' in mode:
                    self.robot_write_commands(msg.name, points[x].positions)
                elif mode == 'velocity':
                    self.robot_write_commands(msg.name, points[x].velocities)
                elif mode == 'pwm' or mode == 'current':
                    self.robot_write_commands(msg.name, points[x].effort)
            elif msg.cmd_type == 'single':
                if 'position' in mode:
                    self.robot_write_joint_command(msg.name, points[x].positions[0])
                elif mode == 'velocity':
                    self.robot_write_joint_command(msg.name, points[x].velocities[0])
                elif mode == 'pwm' or mode == 'current':
                    self.robot_write_joint_command(msg.name, points[x].effort[0])
            if x < len(points) - 1:
                # TODO: clean this up
                period = (
                    Duration.from_msg(points[x].time_from_start).nanoseconds
                    - (self.get_clock().now() - time_start).nanoseconds) / S_TO_NS
                rate = self.create_rate(1.0 / period)
                rate.sleep()
                rate.destroy()
            self.robot_update_joint_states()

        self.exectue_joint_traj = False

    def robot_srv_torque_enable(
        self,
        req: TorqueEnable.Request,
        res: TorqueEnable.Response
    ) -> TorqueEnable.Response:
        """
        Simulate torquing the joints on the robot on/off from a ROS service.

        :param req: TorqueEnable service message request
        :param res: TorqueEnable service message response
        :return: TorqueEnable service message response
        :details: refer to the service definition for details
        """
        if (req.cmd_type == 'group'):
            if (req.enable):
                LOGGER.info(f"The '{req.name}' group was torqued on.")
            else:
                LOGGER.info(f"The '{req.name}' group was torqued off.")
        else:
            if (req.enable):
                LOGGER.info(f"The '{req.name}' joint was torqued on.")
            else:
                LOGGER.info(f"The '{req.name}' joint was torqued off.")
        return res

    def robot_srv_reboot_motors(
        self,
        req: Reboot.Request,
        res: Reboot.Response
    ) -> Reboot.Response:
        """
        Simulate rebooting the motors on the robot from a ROS service.

        :param req: Reboot service message request
        :param res: Reboot service message response
        :return: Reboot service message response
        :details: refer to the service definition for details
        """
        if (req.cmd_type == 'group'):
            LOGGER.info(f"The '{req.name}' group was rebooted.")
        else:
            LOGGER.info(f"The '{req.name}' joint was rebooted.")
        return res

    def robot_srv_get_robot_info(
        self,
        req: RobotInfo.Request,
        res: RobotInfo.Response
    ) -> RobotInfo.Response:
        """
        Get information about the robot from a ROS service.

        :param req: RobotInfo service message request
        :param res: RobotInfo service message response
        :return: RobotInfo service message response
        :details: refer to the service definition for details
        """
        joint_names = []
        if (req.cmd_type == 'group'):
            joint_names = self.group_map[req.name]['joint_names']
            res.profile_type = self.group_map[req.name]['profile_type']
            res.mode = self.group_map[req.name]['mode']
        elif (req.cmd_type == 'single'):
            joint_names.append(req.name)
            res.profile_type = self.motor_map[req.name]['profile_type']
            res.mode = self.motor_map[req.name]['mode']

        res.num_joints = len(joint_names)

        for name in joint_names:
            res.joint_ids.append(self.motor_map[name]['motor_id'])
            if name in self.gripper_map:
                res.joint_sleep_positions.append(
                    self.robot_convert_angular_position_to_linear(name, 0)
                )
                name = self.gripper_map[name]['left_finger']
                res.joint_names.append(name)
            else:
                res.joint_sleep_positions.append(self.sleep_map[name])
                res.joint_names.append(name)
            res.joint_state_indices.append(self.js_index_map[name])
            if self.rd is not None:
                joint_object = next(
                    (joint for joint in self.rd.joints if joint.name == name),
                    None
                )
                res.joint_lower_limits.append(joint_object.limit.lower)
                res.joint_upper_limits.append(joint_object.limit.upper)
                res.joint_velocity_limits.append(joint_object.limit.velocity)

        if 'all' not in req.name:
            res.name.append(req.name)
        else:
            for key, _ in self.group_map.items():
                res.name.append(key)
        return res

    def robot_srv_set_operating_modes(
        self,
        req: OperatingModes.Request,
        res: OperatingModes.Response
    ) -> OperatingModes.Response:
        """
        Change operating modes through a ROS service.

        :param req: OperatingModes service message request
        :param res: OperatingModes service message response
        :return: OperatingModes service message response
        :details: refer to the service definition for details
        """
        self.robot_set_operating_modes(
            req.cmd_type,
            req.name,
            req.mode,
            req.profile_type,
            req.profile_velocity,
            req.profile_acceleration
        )
        return res

    def robot_srv_set_motor_pid_gains(
        self,
        req: MotorGains.Request,
        res: MotorGains.Response
    ) -> MotorGains.Response:
        """
        Set the motor firmware PID gains from a ROS service.

        :param req: MotorGains service message request
        :param res: MotorGains service message response
        :return: MotorGains service message response
        :details: refer to the service defintion for details
        """
        return res

    def robot_srv_set_motor_registers(
        self,
        req: RegisterValues.Request,
        res: RegisterValues.Response
    ) -> RegisterValues.Response:
        """
        Change a register to a value for multiple motors from a ROS service.

        :param req: RegisterValues service message request
        :param res: RegisterValues service message response
        :return: RegisterValues service message response
        :details: refer to the service definition for details - only works with the
            'Profile_Velocity' and 'Profile_Acceleration' registers; otherwise, it doesn't do
            anything
        """
        if req.reg == 'Profile_Velocity' or req.reg == 'Profile_Acceleration':
            reg = 'profile_velocity' if req.reg == 'Profile_Velocity' else 'profile_acceleration'
            if req.cmd_type == 'group':
                self.group_map[req.name][reg] = req.value
                for joint in self.group_map[req.name]['joint_names']:
                    self.motor_map[joint][reg] = req.value
            elif req.cmd_type == 'single':
                self.motor_map[req.name][reg] = req.value
        return res

    def robot_srv_get_motor_registers(
        self,
        req: RegisterValues.Request,
        res: RegisterValues.Response
    ) -> RegisterValues.Response:
        """
        ROS Service that allows the user to read a specific register on multiple motors.

        :param req: RegisterValues service message request
        :param res: RegisterValues service message response
        :return: RegisterValues service message response
        :details: refer to the service definition for details - only works with the
            'Profile_Velocity' and 'Profile_Acceleration' registers; otherwise, it doesn't do
            anything
        """
        if req.reg == 'Profile_Velocity' or req.reg == 'Profile_Acceleration':
            reg = 'profile_velocity' if req.reg == 'Profile_Velocity' else 'profile_acceleration'
            if req.cmd_type == 'group':
                for joint in self.group_map[req.name]['joint_names']:
                    res.values.append(self.motor_map[joint][reg])
            elif req.cmd_type == 'single':
                res.values.append(self.motor_map[req.name][reg])
        return res

    def robot_update_joint_states(self) -> None:
        """
        ROS Timer that updates the joint states based on the current 'self.commands'.

        :details: if a joint is in any form of 'position' mode and its profile_velocity is greater
            than 'self.move_thresh' milliseconds, then 'self.commands[joint]' is a list of
            waypoints (in reverse); that way, the latest position to execute can just be 'popped'
            off the end of the list - which is more efficient; when the last element is taken out
            from the list, the dictionary key is thrown out. Otherwise, 'self.commands[joint]' is a
            scalar. If the scalar is a position, the dictionary key is thrown out afterwards.
            Otherwise, the dictionary key is not thrown out unless its value is 0 (which is what
            you want for 'pwm', 'current', or 'velocity' modes)
        """
        with self.cmd_mutex:
            for joint in self.commands.copy():
                mode = self.motor_map[joint]['mode']

                if 'position' in mode:
                    if isinstance(self.commands[joint], list):
                        value = self.commands[joint].pop()
                        if len(self.commands[joint]) == 0:
                            del self.commands[joint]
                    else:
                        value = self.commands[joint]
                        del self.commands[joint]

                    if joint in self.gripper_map:
                        gpr = self.gripper_map[joint]
                        if mode == 'linear_position':
                            angle = self.robot_convert_linear_position_to_radian(joint, value)
                            self.joint_states.position[self.js_index_map[joint]] = angle
                            self.joint_states.position[
                                self.js_index_map[gpr['left_finger']]] = value / 2.0
                            self.joint_states.position[
                                self.js_index_map[gpr['right_finger']]] = -value / 2.0
                        else:
                            lin_pos = self.robot_convert_angular_position_to_linear(joint, value)
                            self.joint_states.position[self.js_index_map[joint]] = value
                            self.joint_states.position[
                                self.js_index_map[gpr['left_finger']]
                            ] = lin_pos
                            self.joint_states.position[
                                self.js_index_map[gpr['right_finger']]
                            ] = -lin_pos
                    else:
                        self.joint_states.position[self.js_index_map[joint]] = value

                else:
                    value = self.commands[joint]
                    if value == 0:
                        del self.commands[joint]
                    else:
                        if mode == 'velocity':
                            new_val = value / self.timer_hz
                        else:
                            # treat 'pwm' and 'current' equivalently
                            new_val = value / 2000.0
                        self.joint_states.position[self.js_index_map[joint]] += new_val
                        if joint in self.gripper_map:
                            gpr = self.gripper_map[joint]
                            angle = self.joint_states.position[self.js_index_map[joint]]
                            lin_pos = self.robot_convert_angular_position_to_linear(joint, angle)
                            self.joint_states.position[
                                self.js_index_map[gpr['left_finger']]
                            ] = lin_pos
                            self.joint_states.position[
                                self.js_index_map[gpr['right_finger']]
                            ] = -lin_pos

            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            self.pub_joint_states.publish(self.joint_states)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    try:
        node = InterbotixRobotXS()
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            print('Keyboard interrupt caught. Shutting down safely...')
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        print('Keyboard interrupt caught. Shutting down safely...')
    finally:
        if executor is not None:
            if not executor._is_shutdown:
                executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
