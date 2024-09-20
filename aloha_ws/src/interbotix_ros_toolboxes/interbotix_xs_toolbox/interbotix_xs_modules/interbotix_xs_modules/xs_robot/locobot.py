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

"""
Contains classes used to control the Interbotix X-Series LoCoBots.

These classes can be used to control a Interbotix X-Series LoCoBots using Python.
"""

from enum import Enum
from threading import Thread
import time

from interbotix_xs_modules.xs_robot.arm import InterbotixArmXSInterface
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
from interbotix_xs_modules.xs_robot.turret import InterbotixTurretXSInterface
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity


class BaseType(Enum):
    """Enum representing the available base types for an Interbotix LoCoBot."""

    KOBUKI = 0
    CREATE3 = 1


class InterbotixLocobotXS:
    """Standalone Module to control an Interbotix LoCoBot."""

    def __init__(
        self,
        robot_model: str,
        arm_model: str = None,
        use_base: bool = False,
        use_perception: bool = False,
        use_armtag: bool = False,
        base_type: BaseType = BaseType.CREATE3,
        arm_group_name: str = 'arm',
        gripper_name: str = 'gripper',
        turret_group_name: str = 'camera',
        robot_name: str = 'locobot',
        topic_dxl_joint_states: str = 'dynamixel/joint_states',
        topic_base_joint_states: str = 'mobile_base/joint_states',
        use_nav: bool = False,
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        node_name: str = 'robot_manipulation',
        start_on_init: bool = True,
        args=None,
    ):
        """
        Construct the Standalone InterbotixLocobotXS object.

        :param robot_model: Interbotix Locobot model (ex. 'locobot_px100' or 'locobot_base')
        :param arm_model: (optional) the model of arm used on the LoCoBot. defaults to `None`
        :param use_base: (optional) `True` to use the LoCoBot's base; `False` otherwise. defaults
            to `False`
        :param use_base: (optional) `True` to use perception; `False` otherwise. defaults to
            `False`
        :param use_base: (optional) `True` to use the LoCoBot's armtag; `False` otherwise. defaults
            to `False`
        :param base_type: (optional) the base type of the LoCoBot. defaults to `BaseType.CREATE3`
        :param arm_group_name: (optional) joint group name that contains the 'arm' joints as
            defined in the 'motor_config' yaml file
        :param gripper_name: (optional) name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param turret_group_name: (optional) joint group name that contains the 'turret' joints as
            defined in the 'motor_config' yaml file; typically, this is 'camera'
        :param robot_name: (optional) this can be customized if controlling two or more LoCoBots
            from one computer (like 'locobot1' and 'locobot2'); defaults to `'locobot'`
        :param topic_dxl_joint_states: (optional) name of the joint states topic that contains just
            the states of the dynamixel servos
        :param topic_base_joint_states: (optional) name of the joints states topic that contains
            the states of the base. defaults to `'mobile_base/joint_states'`
        :param use_nav: (optional) whether or not to enable navigation features. requires that nav2
            be launched. defaults to `False`
        :param logging_level: (optional) rclpy logging severity level. Can be DEBUG, INFO, WARN,
            ERROR, or FATAL. defaults to INFO
        :param node_name: (optional) name to give to the core started by this class, defaults to
            'robot_manipulation'
        :param start_on_init: (optional) set to `True` to start running the spin thread after the
            object is built; set to `False` if intending to sub-class this. If set to `False`,
            either call the `start()` method later on, or add the core to an executor in another
            thread.
        """
        self.core = InterbotixRobotXSCore(
            robot_model=robot_model,
            robot_name=robot_name,
            topic_joint_states=topic_dxl_joint_states,
            logging_level=logging_level,
            node_name=node_name,
            args=args
        )
        self.camera = InterbotixTurretXSInterface(
            core=self.core,
            turret_name=turret_group_name
        )
        if use_base:
            if base_type == BaseType.KOBUKI:
                from interbotix_xs_modules.xs_robot.kobuki import InterbotixKobukiInterface
                self.base = InterbotixKobukiInterface(
                    core=self.core,
                    robot_name=robot_name,
                    topic_base_joint_states=topic_base_joint_states,
                    use_nav=use_nav,
                )
            elif base_type == BaseType.CREATE3:
                from interbotix_xs_modules.xs_robot.create3 import InterbotixCreate3Interface
                self.base = InterbotixCreate3Interface(
                    core=self.core,
                    robot_name=robot_name,
                    topic_base_joint_states=topic_base_joint_states,
                    use_nav=use_nav,
                )
        if use_perception:
            from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
            self.pcl = InterbotixPointCloudInterface(
                filter_ns=f'{robot_name}/pc_filter'
            )
        if arm_model is not None:
            self.arm = InterbotixArmXSInterface(
                core=self.core,
                robot_model=arm_model,
                group_name=arm_group_name
            )
            self.gripper = InterbotixGripperXSInterface(
                core=self.core,
                gripper_name=gripper_name
            )
            if use_armtag:
                from interbotix_perception_modules.armtag import InterbotixArmTagInterface
                self.armtag = InterbotixArmTagInterface(
                    armtag_ns=f'{robot_name}/armtag',
                    apriltag_ns=f'{robot_name}/apriltag',
                    init_node=False
                )

        if start_on_init:
            self.start()

    def start(self) -> None:
        """Start a background thread that builds and spins an executor."""
        self._execution_thread = Thread(target=self.run)
        self._execution_thread.start()

    def run(self) -> None:
        """Thread target."""
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.core)
        self.ex.spin()

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        self.core.destroy_node()
        rclpy.shutdown()
        self._execution_thread.join()
        time.sleep(0.5)
