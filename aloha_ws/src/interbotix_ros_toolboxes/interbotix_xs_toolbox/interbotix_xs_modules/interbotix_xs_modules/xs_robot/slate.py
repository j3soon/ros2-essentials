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

"""Contains classes used to control the Interbotix Slate mobile base."""

from interbotix_common_modules.common_robot.robot import InterbotixRobotNode
from interbotix_slate_msgs.srv import SetString
from interbotix_xs_modules.xs_robot.mobile_base import InterbotixMobileBaseInterface
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool


class InterbotixSlate:
    def __init__(
        self,
        robot_name: str = None,
        node: InterbotixRobotNode = None,
        enable_torque_on_init: bool = True,
    ) -> None:
        """
        Construct an InterbotixSlate object.

        :param robot_name: (optional) namespace of the Slate base nodes. defaults to `None`
        :param node: reference to the InterbotixRobotNode class containing the internal ROS
            plumbing that drives the Python API
        :param enable_torque_on_init: (optional) `True` to enable base torque on init, `False` to
            disable. defaults to `True`
        """
        self.base = InterbotixSlateInterface(
            core=node,
            robot_name=robot_name,
            enable_torque_on_init=enable_torque_on_init,
        )


class InterbotixSlateInterface(InterbotixMobileBaseInterface):
    """Definition of the Interbotix Slate Module."""

    def __init__(
        self,
        core: InterbotixRobotNode,
        robot_name: str,
        enable_torque_on_init: bool = True,
        topic_base_joint_states: str = 'mobile_base/joint_states',
        topic_cmd_vel: str = 'mobile_base/cmd_vel',
        nav_timeout_sec: float = 300.0,
        use_nav: bool = False,
    ):
        """
        Construct the InterbotixSlateInterface object.

        :param core: reference to the InterbotixRobotNode class containing the internal ROS
            plumbing that drives the Python API
        :param robot_name: namespace of the Slate nodes
        :param enable_torque_on_init: (optional) `True` to enable base torque on init, `False` to
            disable. defaults to `True`
        :param topic_base_joint_states: (optional) name of the joints states topic that contains
            the states of the Slate's two wheels. defaults to `'mobile_base/joint_states'`
        :param topic_cmd_vel: (optional) name of the twist topic to which velocity commands should
            be published. defaults to `'mobile_base/cmd_vel'`
        :param nav_timeout_sec: (optional) length of time in seconds after which to cancel a
            navigation goal. defaults to `300` (five minutes)
        :param use_nav: (optional) whether or not to enable navigation features. requires that nav2
            be launched. defaults to `False`
        """
        self.robot_node = core
        super().__init__(
            core=core,
            robot_name=robot_name,
            topic_base_joint_states=topic_base_joint_states,
            topic_cmd_vel=topic_cmd_vel,
            nav_timeout_sec=nav_timeout_sec,
            use_nav=use_nav,
        )

        cb_group_slate = ReentrantCallbackGroup()

        self.client_set_text = self.robot_node.create_client(
            srv_type=SetString,
            srv_name='mobile_base/set_text',
            callback_group=cb_group_slate,
        )
        self.client_set_motor_torque = self.robot_node.create_client(
            srv_type=SetBool,
            srv_name='mobile_base/set_motor_torque_status',
            callback_group=cb_group_slate,
        )

        while (
            not self.client_set_motor_torque.wait_for_service(timeout_sec=5.0)
            and not self.client_set_text.wait_for_service(timeout_sec=5.0)
            and rclpy.ok()
        ):
            self.robot_node.get_logger().error(
                (
                    "Failed to find services under namespace 'mobile_base'. Is the slate "
                    'base driver node running under that namespace?'
                )
            )

        self._torque_status = enable_torque_on_init
        self.set_motor_torque(enable=enable_torque_on_init)

        self.robot_node.get_logger().info('Initialized InterbotixSlateInterface!')

    def set_text(self, text: str) -> bool:
        """
        Set the text on the Slate base's screen.

        :param text: Text to set
        :return: `True` if the set_text service succeeded, `False` otherwise
        """
        future = self.client_set_text.call_async(SetString.Request(data=text))
        self.robot_node.wait_until_future_complete(future)
        result: SetString.Response = future.result()
        self.robot_node.get_logger().info(result.message)
        return result.success

    def set_motor_torque(self, enable: bool) -> bool:
        """
        Set the Slate base's motor torque status.

        :param enable: `True` to enable torque on the base's motors, `False` to disable
        :return: `True` if the set_motor_torque service succeeded, `False` otherwise
        """
        future = self.client_set_motor_torque.call_async(
            SetBool.Request(data=enable)
        )
        self.robot_node.wait_until_future_complete(future)
        result: SetBool.Response = future.result()
        self.robot_node.get_logger().info(result.message)
        if result.success:
            self._torque_status = enable
        return result.success

    def get_torque_status(self) -> bool:
        """
        Get the Slate base's motor torque status.

        :return: The Slate base's motor torque status.
        """
        return self._torque_status

    def play_sound(self, *args, **kwargs) -> None:
        raise NotImplementedError()

    def reset_odom(self, *args, **kwargs) -> None:
        raise NotImplementedError()
