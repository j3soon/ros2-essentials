# Copyright 2024 Trossen Robotics
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
#    * Neither the name of the copyright holder nor the names of its
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

"""Contains the `InterbotixRobotXSCore` class that interfaces with the interbotix_xs_sdk."""

import copy

from threading import Lock
from typing import (
    Dict,
    List,
    Optional,
)

from interbotix_xs_msgs.msg import (
    JointGroupCommand,
    JointSingleCommand,
    JointTrajectoryCommand
)
from interbotix_xs_msgs.srv import (
    MotorGains,
    OperatingModes,
    Reboot,
    RegisterValues,
    RobotInfo,
    TorqueEnable,
)
from interbotix_common_modules.common_robot.robot import (
    InterbotixRobotNode,
    create_interbotix_global_node,
)
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity, set_logger_level
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class InterbotixRobotXSCore:
    """Class that interfaces with the xs_sdk node ROS interfaces."""

    joint_states: Optional[JointState]
    robot_node: InterbotixRobotNode

    def __init__(
        self,
        robot_model: str,
        robot_name: Optional[str] = None,
        topic_joint_states: str = 'joint_states',
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        node_name: str = 'interbotix_robot_manipulation',
        node: Optional[InterbotixRobotNode] = None,
        args=None,
    ) -> None:
        """
        Construct the InterbotixRobotXSCore object.

        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'vx300s')
        :param robot_name: (optional) defaults to value given to 'robot_model'; this can be
            customized if controlling two of the same arms from one computer (like 'arm1/wx200' and
            'arm2/wx200')
        :param topic_joint_states: (optional) the specific JointState topic output by the xs_sdk
            node
        :param logging_level: (optional) rclpy logging severity level. Can be DEBUG, INFO, WARN,
            ERROR, or FATAL. defaults to INFO
        :param node_name: (optional) name to give to the node started by this class, defaults to
            'interbotix_robot_manipulation'
        :param node: (optional) the InterbotixRobotNode to base this class's ROS components on.
        """
        self.robot_model = robot_model
        self.robot_name = robot_name
        self.node_name = node_name

        # Set robot_name to robot_model if unspecified
        if self.robot_name is None:
            self.robot_name = robot_model

        # Set ns to empty string if unspecified to prevent invalid namespace errors
        if self.robot_name == '':
            self.ns = ''
        else:
            self.ns = f'/{self.robot_name}'

        if node is None:
            self.robot_node = create_interbotix_global_node()
        else:
            self.robot_node = node

        set_logger_level(self.node_name, logging_level)

        self.robot_node.get_logger().debug((
            f"Created node with name='{self.node_name}' in namespace='{robot_name}'"
        ))

        self.topic_joint_states = topic_joint_states
        self.joint_states: JointState = None
        self.js_mutex = Lock()

        cb_group_dxl_core = ReentrantCallbackGroup()

        self.srv_set_op_modes = self.robot_node.create_client(
            srv_type=OperatingModes,
            srv_name=f'{self.ns}/set_operating_modes',
            callback_group=cb_group_dxl_core,
        )
        self.srv_set_pids = self.robot_node.create_client(
            srv_type=MotorGains,
            srv_name=f'{self.ns}/set_motor_pid_gains',
            callback_group=cb_group_dxl_core,
        )
        self.srv_set_reg = self.robot_node.create_client(
            srv_type=RegisterValues,
            srv_name=f'{self.ns}/set_motor_registers',
            callback_group=cb_group_dxl_core,
        )
        self.srv_get_reg = self.robot_node.create_client(
            srv_type=RegisterValues,
            srv_name=f'{self.ns}/get_motor_registers',
            callback_group=cb_group_dxl_core,
        )
        self.srv_get_info = self.robot_node.create_client(
            srv_type=RobotInfo,
            srv_name=f'{self.ns}/get_robot_info',
            callback_group=cb_group_dxl_core,
        )
        self.srv_torque = self.robot_node.create_client(
            srv_type=TorqueEnable,
            srv_name=f'{self.ns}/torque_enable',
            callback_group=cb_group_dxl_core,
        )
        self.srv_reboot = self.robot_node.create_client(
            srv_type=Reboot,
            srv_name=f'{self.ns}/reboot_motors',
            callback_group=cb_group_dxl_core,
        )

        # Check for xs_sdk by looking for set_operating_modes
        while not self.srv_set_op_modes.wait_for_service(timeout_sec=5.0) and rclpy.ok():
            self.robot_node.get_logger().error(
                f"Failed to find services under namespace '{self.ns}'. Is the xs_sdk "
                'running under that namespace?'
            )
        self.srv_set_pids.wait_for_service()
        self.srv_set_reg.wait_for_service()
        self.srv_get_reg.wait_for_service()
        self.srv_get_info.wait_for_service()
        self.srv_torque.wait_for_service()
        self.srv_reboot.wait_for_service()
        self.pub_group = self.robot_node.create_publisher(
            msg_type=JointGroupCommand,
            topic=f'{self.ns}/commands/joint_group',
            qos_profile=10,
            callback_group=cb_group_dxl_core
        )
        self.pub_single = self.robot_node.create_publisher(
            msg_type=JointSingleCommand,
            topic=f'{self.ns}/commands/joint_single',
            qos_profile=10,
            callback_group=cb_group_dxl_core
        )
        self.pub_traj = self.robot_node.create_publisher(
            msg_type=JointTrajectoryCommand,
            topic=f'{self.ns}/commands/joint_trajectory',
            qos_profile=10,
            callback_group=cb_group_dxl_core
        )
        self.sub_joint_states = self.robot_node.create_subscription(
            msg_type=JointState,
            topic=f'{self.ns}/{self.topic_joint_states}',
            callback=self._joint_state_cb,
            qos_profile=10,
            callback_group=cb_group_dxl_core,
        )
        self.robot_node.get_logger().debug(
            f'Trying to find joint states on topic "{self.ns}/{self.topic_joint_states}"...'
        )
        while self.joint_states is None and rclpy.ok():
            rclpy.spin_once(self.robot_node)
        self.robot_node.get_logger().debug('Found joint states. Continuing...')

        self.js_index_map = dict(
            zip(self.joint_states.name, range(len(self.joint_states.name)))
        )
        self.robot_node.get_logger().info(
            '\n'
            f'\tRobot Name: {self.robot_name}\n'
            f'\tRobot Model: {self.robot_model}'
        )
        self.robot_node.get_logger().info('Initialized InterbotixRobotXSCore!')

    def get_node(self) -> InterbotixRobotNode:
        return self.robot_node

    def robot_set_operating_modes(
        self,
        cmd_type: str,
        name: str,
        mode: str,
        profile_type: str = 'velocity',
        profile_velocity: int = 0,
        profile_acceleration: int = 0,
    ) -> None:
        """
        Set the operating mode for either a single motor or a group of motors.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param mode: desired operating mode like "position" or "velocity".
        :param profile_type: (optional) can be "time" or "velocity".
        :param profile_velocity: (optional) passthrough to the Profile_Velocity register.
        :param profile_acceleration: (optional) passthrough to the Profile_Acceleration register.
        :details: See the OperatingModes Service description for all choices
        """
        future = self.srv_set_op_modes.call_async(
            OperatingModes.Request(
                cmd_type=cmd_type,
                name=name,
                mode=mode,
                profile_type=profile_type,
                profile_velocity=profile_velocity,
                profile_acceleration=profile_acceleration,
            )
        )
        self.get_node().wait_until_future_complete(future)

    def robot_set_motor_pid_gains(
        self,
        cmd_type: str,
        name: str,
        kp_pos: int,
        ki_pos: int = 0,
        kd_pos: int = 0,
        k1: int = 0,
        k2: int = 0,
        kp_vel: int = 100,
        ki_vel: int = 1920,
    ) -> None:
        """
        Set the internal PID gains for either a single motor or a group of motors.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param kp_pos: passthrough to the Position_P_Gain register.
        :param ki_pos: (optional) passthrough to the Position_I_Gain register.
        :param kd_pos: (optional) passthrough to the Position_D_Gain register.
        :param k1: (optional) passthrough to the Feedforward_1st_Gain register.
        :param k2: (optional) passthrough to the Feedforward_2nd_Gain register.
        :param kp_vel: (optional) passthrough to the Velocity_P_Gain register.
        :param ki_vel: (optional) passthrough to the Velocity_I_Gain register.
        :details: See the MotorGains Service description for details
        """
        future = self.srv_set_pids.call_async(
            MotorGains.Request(
                cmd_type=cmd_type,
                name=name,
                kp_pos=kp_pos,
                ki_pos=ki_pos,
                kd_pos=kd_pos,
                k1=k1,
                k2=k2,
                kp_vel=kp_vel,
                ki_vel=ki_vel,
            )
        )
        self.get_node().wait_until_future_complete(future)

    def robot_set_motor_registers(
        self, cmd_type: str, name: str, reg: str, value: int
    ) -> None:
        """
        Set the desired register for either a single motor or a group of motors.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param reg: desired register name
        :param value: desired value for the above register
        """
        future = self.srv_set_reg.call_async(
            RegisterValues.Request(cmd_type=cmd_type, name=name, reg=reg, value=value)
        )
        self.get_node().wait_until_future_complete(future)

    def robot_get_motor_registers(self, cmd_type: str, name: str, reg: str) -> List[int]:
        """
        Get the desired register value from either a single motor or a group of motors.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param reg: desired register name
        :return: list of register values
        """
        future = self.srv_get_reg.call_async(
            RegisterValues.Request(cmd_type=cmd_type, name=name, reg=reg)
        )
        self.get_node().wait_until_future_complete(future)
        return future.result().values

    def robot_get_robot_info(self, cmd_type: str, name: str) -> RobotInfo.Response:
        """
        Get information about the robot - mostly joint limit data.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :return: an object with the same structure as a RobotInfo Service description
        """
        future = self.srv_get_info.call_async(
            RobotInfo.Request(cmd_type=cmd_type, name=name)
        )
        self.get_node().wait_until_future_complete(future)
        return future.result()

    def robot_torque_enable(self, cmd_type: str, name: str, enable: bool) -> None:
        """
        Torque a single motor or a group of motors to be on or off.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param enable: `True` to torque on or `False` to torque off
        """
        future = self.srv_torque.call_async(
            TorqueEnable.Request(cmd_type=cmd_type, name=name, enable=enable)
        )
        self.get_node().wait_until_future_complete(future)

    def robot_reboot_motors(
        self,
        cmd_type: str,
        name: str,
        enable: bool,
        smart_reboot: bool = False,
    ):
        """
        Reboot a single motor or a group of motors if they are in an error state.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param enable: `True` to torque on or `False` to leave torqued off after rebooting
        :param smart_reboot: (optional) if `cmd_type` is set to 'group', setting this to `True`
            will only reboot those motors that are in an error state (as opposed to all motors
            within the group regardless of if they are in an error state)
        """
        future = self.srv_reboot.call_async(
            Reboot.Request(cmd_type=cmd_type, name=name, enable=enable, smart_reboot=smart_reboot)
        )
        self.get_node().wait_until_future_complete(future)

    def robot_write_commands(self, group_name: str, commands: List[float]) -> None:
        """
        Command a group of motors.

        :param group_name: the group name of the motors to command
        :param commands: desired list of commands
        :details: refer to the JointGroupCommand Message description for more info
        """
        self.pub_group.publish(JointGroupCommand(name=group_name, cmd=commands))

    def robot_write_joint_command(self, joint_name: str, command: float) -> None:
        """
        Command a single motor.

        :param joint_name: the name of the motor to command
        :param command: desired command
        :details: refer to the JointSingleCommand Message description for more info
        """
        self.pub_single.publish(JointSingleCommand(name=joint_name, cmd=command))

    def robot_write_trajectory(
        self,
        cmd_type: str,
        name: str,
        traj_type: str,
        raw_traj: List[Dict[float, List[float]]]
    ) -> None:
        """
        Command a trajectory (position or velocity) to a single motor or a group of motors.

        :param cmd_type: can be "group" for a group of motors or "single" for a single motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type` is
            'single'
        :param traj_type: "position" if the trajectory is a list of positions [rad]; otherwise
            "velocity" if the trajectory is a list of velocities [rad/s]
        :param raw_traj: list of dictionaries where each dictionary is made up of a float / list of
            float pairs. the 'key' is the desired time [sec] from start that the 'value' (list of
            floats) should be executed.
        :details: an example input trajectory for a pan/tilt mechanism could look like:
            [{0.0, [1.0,  1.0]   },
             {1.5, [-1.0, 0.75]  },
             {2.3, [0.0,  0.0]   }]
        :details: refer to the JointTrajectoryCommand Message description for more info
        """
        traj = JointTrajectory()
        for point in raw_traj:
            for key, value in point.items():
                traj_point = JointTrajectoryPoint()
                if traj_type == 'position':
                    traj_point.positions = value
                elif traj_type == 'velocity':
                    traj_point.velocities = value
                traj_point.time_from_start = Duration(seconds=key).to_msg()
                traj.points.append(traj_point)
        self.pub_traj.publish(JointTrajectoryCommand(cmd_type=cmd_type, name=name, traj=traj))

    def robot_get_joint_states(self) -> JointState:
        """
        Get the current joint states (position, velocity, effort) of all DYNAMIXEL motors.

        :return: JointState ROS message. Refer to online documentation to see its structure
        """
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        return joint_states

    def robot_get_single_joint_state(self, name: str) -> Dict[str, List[float]]:
        """
        Get a single joint state for the specified DYNAMIXEL motor.

        :param name: desired motor name for which to get the joint state
        :return: dictionary with 3 keys: "position", "velocity", and "effort". Units are rad,
            rad/s, and mA
        """
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        joint_index = joint_states.name.index(name)
        joint_info = {}
        joint_info['position'] = joint_states.position[joint_index]
        joint_info['velocity'] = joint_states.velocity[joint_index]
        joint_info['effort'] = joint_states.effort[joint_index]
        return joint_info

    def _joint_state_cb(self, msg: JointState):
        """
        Get the latest JointState message through a ROS Subscriber Callback.

        :param msg: JointState message
        """
        with self.js_mutex:
            self.joint_states = msg
