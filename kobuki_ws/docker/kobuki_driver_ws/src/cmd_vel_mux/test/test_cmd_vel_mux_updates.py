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

import collections
import contextlib
import functools
import re
import unittest

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch_ros.substitutions
import launch_testing.actions
import launch_testing.markers
import launch_testing.tools

import rcl_interfaces.srv
import rclpy


def flatten(d, prefix=None):
    assert isinstance(d, collections.abc.Mapping)

    flat_d = {}
    for k, v in d.items():
        if prefix is not None:
            k = prefix + '.' + k
        if isinstance(v, collections.abc.Mapping):
            flat_d.update(flatten(v, prefix=k))
        else:
            flat_d[k] = v
    return flat_d


@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ])


class TestCmdVelMux(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_cmd_vel_mux_node(self, *, parameters):
            cmd_vel_mux_node_action = launch_ros.actions.Node(
                package='cmd_vel_mux',
                executable='cmd_vel_mux_node',
                name='cmd_vel_mux',
                parameters=[parameters],
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, cmd_vel_mux_node_action, proc_info, proc_output
            ) as node_process:
                yield node_process
        cls.launch_cmd_vel_mux_node = launch_cmd_vel_mux_node

        rclpy.init()
        cls._node = rclpy.create_node(
            'cmd_vel_mux_testing_node', start_parameter_services=False
        )
        cls._cmd_vel_mux_set_parameters_client = cls._node.create_client(
            rcl_interfaces.srv.SetParametersAtomically,
            '/cmd_vel_mux/set_parameters_atomically'
        )

    def update_cmd_vel_mux_params(self, *, parameters):
        self.assertTrue(self._cmd_vel_mux_set_parameters_client.wait_for_service(timeout_sec=5))

        request = rcl_interfaces.srv.SetParametersAtomically.Request()
        print(flatten(parameters))
        request.parameters = [
            rclpy.Parameter(name=name, value=value).to_parameter_msg()
            for name, value in flatten(parameters).items()
        ]
        future = self._cmd_vel_mux_set_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        response = future.result()
        return response.result.successful, response.result.reason

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_client(cls._cmd_vel_mux_set_parameters_client)
        cls._node.destroy_node()
        rclpy.shutdown()

    def test_no_params(self):
        with self.launch_cmd_vel_mux_node(parameters={}) as node_process:
            self.assertTrue(node_process.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*WARN.*No subscribers configured!')
                ], strict=False
            ), timeout=2))

    def test_invalid_subscriber(self):
        with self.launch_cmd_vel_mux_node(
            parameters={
                'subscribers': {
                    'some_invalid_subscriber': ''
                }
            }
        ) as node_process:
            self.assertTrue(node_process.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*std::runtime_error.*'),
                    re.compile(r'.*Invalid parameters.*')
                ], strict=False
            ), timeout=2))
        self.assertTrue(node_process.wait_for_shutdown(timeout=2))
        self.assertNotEqual(node_process.exit_code, launch_testing.asserts.EXIT_OK)

    def test_invalid_subscriber_params(self):
        with self.launch_cmd_vel_mux_node(
            parameters={
                'subscribers': {
                    'some_subscriber': {
                        'topic': 0
                    }
                }
            }
        ) as node_process:
            self.assertTrue(node_process.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*std::runtime_error.*'),
                    re.compile(r'.*Invalid parameters.*')
                ], strict=False
            ), timeout=2))
        self.assertTrue(node_process.wait_for_shutdown(timeout=2))
        self.assertNotEqual(node_process.exit_code, launch_testing.asserts.EXIT_OK)

    def test_incomplete_params(self):
        with self.launch_cmd_vel_mux_node(
            parameters={
                'subscribers': {
                    'some_subscriber': {
                        'topic': 'some_topic',
                        'priority': 0,
                        'short_desc': ''
                    }
                }
            }
        ) as node_process:
            self.assertTrue(node_process.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*std::runtime_error.*'),
                    re.compile(r'.*Incomplete parameters.*'),
                ], strict=False
            ), timeout=2))
        self.assertTrue(node_process.wait_for_shutdown(timeout=2))
        self.assertNotEqual(node_process.exit_code, launch_testing.asserts.EXIT_OK)

    def test_params_update(self):
        with self.launch_cmd_vel_mux_node(parameters={
            'subscribers': {
                'some_subscriber': {
                    'topic': 'some_topic',
                    'priority': 0,
                    'timeout': 1.0,
                    'short_desc': 'A dummy description'
                }
            }
        }) as node_process:
            self.assertTrue(node_process.wait_for_output(functools.partial(
                launch_testing.tools.expect_output, expected_lines=[
                    re.compile(r'.*CmdVelMux : \(re\)configured.*'),
                ], strict=False
            ), timeout=2))

            ok, reason = self.update_cmd_vel_mux_params(parameters={
                'subscribers': {
                    'some_invalid_subscriber': ''
                }
            })
            self.assertFalse(ok)
            self.assertEqual(reason, 'Invalid or unknown parameter')

            ok, reason = self.update_cmd_vel_mux_params(parameters={
                'subscribers': {
                    'another_subscriber': {
                        'topic': 'another_topic',
                        'priority': 0,
                        'timeout': 0.1,
                    }
                }
            })
            self.assertFalse(ok)
            self.assertEqual(reason, 'Incomplete parameters')

            ok, reason = self.update_cmd_vel_mux_params(parameters={
                'subscribers': {
                    'another_subscriber': {
                        'topic': 0,
                    }
                }
            })
            self.assertFalse(ok)
            self.assertEqual(reason, 'Invalid parameter')

            ok, _ = self.update_cmd_vel_mux_params(parameters={
                'subscribers': {
                    'another_subscriber': {
                        'topic': 'another_topic',
                        'priority': 1,
                        'timeout': 0.1,
                        'short_desc': 'Another dummy description'
                    }
                }
            })
            self.assertTrue(ok)
