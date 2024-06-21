# Copyright 2020 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of {copyright_holder} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import collections.abc
import contextlib
import unittest

import geometry_msgs.msg
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.substitutions
import launch_testing.actions
import launch_testing.tools
import rclpy
import rclpy.qos
import std_msgs.msg


def generate_test_description():
    path_to_cmd_vel_mux_params = launch.substitutions.PathJoinSubstitution([
        launch_ros.substitutions.FindPackageShare(package='cmd_vel_mux'),
        'config',
        'cmd_vel_mux_params.yaml'
    ])
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cmd_vel_mux',
            executable='cmd_vel_mux_node',
            parameters=[path_to_cmd_vel_mux_params],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest()
    ])


latching_qos_profile = rclpy.qos.QoSProfile(depth=1)
latching_qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
latching_qos_profile.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL


class TestCmdVelMux(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        rclpy.init()
        cls._node = rclpy.create_node(
            'cmd_vel_mux_testing_node', start_parameter_services=False
        )

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        rclpy.shutdown()

    @contextlib.contextmanager
    def pub(
        self,
        message,
        *,
        topic,
        rate,
        qos_profile=None
    ):
        if qos_profile is None:
            qos_profile = rclpy.qos.qos_profile_system_default
        publisher = self._node.create_publisher(type(message), topic, qos_profile=qos_profile)
        timer = self._node.create_timer(1.0 / rate, lambda *args: publisher.publish(message))
        try:
            yield
        finally:
            self._node.destroy_timer(timer)
            self._node.destroy_publisher(publisher)

    def expect(
        self,
        expected_sequence,
        *,
        topic,
        message_type=None,
        timeout=None,
        qos_profile=None
    ):
        future = rclpy.task.Future()

        if not isinstance(expected_sequence, collections.abc.Sequence):
            if expected_sequence is not None:
                expected_sequence = [expected_sequence]

        if expected_sequence is not None and len(expected_sequence) > 0:
            if message_type is None:
                message_type = type(expected_sequence[0])
            sequence_types = tuple(type(message) for message in expected_sequence)
            if not all(type_ == message_type for type_ in sequence_types):
                raise ValueError(
                    f'inconsistent message types: found {sequence_types}'
                    f' but expected {message_type} only'
                )

            def _callback(message):
                nonlocal expected_sequence
                if expected_sequence[0] == message:
                    expected_sequence = expected_sequence[1:]
                    if len(expected_sequence) == 0:
                        future.set_result(None)
        else:
            if message_type is None:
                raise ValueError('cannot infer message type')

            def _callback(message):
                future.set_result(None)

        if qos_profile is None:
            qos_profile = rclpy.qos.qos_profile_system_default

        subscription = self._node.create_subscription(
            message_type, topic, _callback, qos_profile=qos_profile
        )
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            return ((expected_sequence is None) ^ future.done())
        finally:
            self._node.destroy_subscription(subscription)

    def test_idle_mux(self):
        expected_active_input = std_msgs.msg.String()
        expected_active_input.data = 'idle'
        self.assertTrue(self.expect(
            expected_active_input,
            topic='active', timeout=2,
            qos_profile=latching_qos_profile
        ))
        self.assertTrue(self.expect(
            None, topic='cmd_vel', message_type=geometry_msgs.msg.Twist, timeout=2
        ))

    def test_mux_with_single_input(self):
        default_twist = geometry_msgs.msg.Twist()
        default_twist.linear.x = 1.0
        with self.pub(default_twist, topic='input/default', rate=20):
            expected_active_input = std_msgs.msg.String()
            expected_active_input.data = 'default_input'
            self.assertTrue(self.expect(
                expected_active_input,
                topic='active', timeout=2,
                qos_profile=latching_qos_profile
            ))
            self.assertTrue(self.expect(default_twist, topic='cmd_vel', timeout=2))

    def test_mux_priority_override(self):
        default_twist = geometry_msgs.msg.Twist()
        default_twist.linear.x = 1.0

        joystick_twist = geometry_msgs.msg.Twist()
        joystick_twist.angular.z = 1.0

        with self.pub(
            default_twist, topic='input/default', rate=20
        ), self.pub(
            joystick_twist, topic='input/joystick', rate=20
        ):
            expected_active_input = std_msgs.msg.String()
            expected_active_input.data = 'navigation_stack_controller'
            self.assertTrue(self.expect(
                expected_active_input,
                topic='active', timeout=2,
                qos_profile=latching_qos_profile
            ))
            self.assertTrue(self.expect(joystick_twist, topic='cmd_vel', timeout=2))

    def test_mux_timeout(self):
        no_twist = geometry_msgs.msg.Twist()
        with self.pub(no_twist, topic='input/default', rate=8):
            idle_input = std_msgs.msg.String()
            idle_input.data = 'idle'
            default_input = std_msgs.msg.String()
            default_input.data = 'default_input'
            self.assertTrue(self.expect(
                [idle_input, default_input, idle_input],
                topic='active', timeout=5,
                qos_profile=latching_qos_profile
            ))
