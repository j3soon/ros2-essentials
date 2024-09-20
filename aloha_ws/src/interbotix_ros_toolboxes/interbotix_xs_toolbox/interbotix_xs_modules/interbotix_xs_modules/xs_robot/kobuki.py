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
Contains classes used to control the Kobuki mobile base for Interbotix X-Series LoCoBots.

These classes can be used to control a Kobuki mobile base for Interbotix X-Series LoCoBots using
Python.
"""

from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_modules.xs_robot.mobile_base import InterbotixMobileBaseInterface
from kobuki_ros_interfaces.msg import Sound
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from std_msgs.msg import Empty


class InterbotixKobukiInterface(InterbotixMobileBaseInterface):
    """Definition of the Interbotix Kobuki Module."""

    def __init__(
        self,
        core: InterbotixRobotXSCore,
        robot_name: str,
        topic_base_joint_states: str = 'mobile_base/joint_states',
        topic_cmd_vel: str = 'mobile_base/commands/velocity',
        nav_timeout_sec: float = 300.0,
        use_nav: bool = False,
    ):
        """
        Construct the InterbotixKobukiInterface object.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        :param robot_name: namespace of the Kobuki node (a.k.a the name of the Interbotix LoCoBot)
        :param topic_base_joint_states: (optional) name of the joints states topic that contains
            the states of the Kobuki's two wheels. defaults to `'mobile_base/joint_states'`
        :param topic_cmd_vel: (optional) name of the twist topic to which velocity commands should
            be published. defaults to `'mobile_base/commands/velocity'`
        :param nav_timeout_sec: (optional) length of time in seconds after which to cancel a
            navigation goal. defaults to `300` (five minutes)
        :param use_nav: (optional) whether or not to enable navigation features. requires that nav2
            be launched. defaults to `False`
        """
        super().__init__(
            core=core,
            robot_name=robot_name,
            topic_base_joint_states=topic_base_joint_states,
            topic_cmd_vel=topic_cmd_vel,
            nav_timeout_sec=nav_timeout_sec,
            use_nav=use_nav,
        )
        self.pub_base_sound = self.core.create_publisher(
            msg_type=Sound,
            topic='commands/sound',
            qos_profile=1,
        )
        self.pub_base_reset_odom = self.core.create_publisher(
            msg_type=Empty,
            topic='commands/reset_odometry',
            qos_profile=1,
        )

        self.core.get_clock().sleep_for(Duration(nanoseconds=int(0.5 * S_TO_NS)))
        self.core.get_logger().info('Initialized InterbotixKobukiInterface!')

    def reset_odom(self) -> None:
        """Reset odometry to zero."""
        self.pub_base_reset_odom.publish(Empty())
        self.play_sound(Sound(value=Sound.CLEANINGEND))

    def play_sound(self, sound: Sound) -> None:
        """
        Publish a sound or sounds using the base.

        :param sound: Sound message to play
        """
        self.pub_base_sound.publish(sound)
