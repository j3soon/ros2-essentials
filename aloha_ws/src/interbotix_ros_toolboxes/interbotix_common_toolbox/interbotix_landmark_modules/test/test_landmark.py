#!/usr/bin/env python3

# Copyright 2023 Trossen Robotics
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

from geometry_msgs.msg import Transform
from interbotix_landmark_modules.landmark import Landmark, LandmarkCollection
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.time import Time
from tf2_geometry_msgs import TransformStamped
import tf2_ros


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    static_transform_pub_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_tf_tools'),
                'launch',
                'static_transform_pub.launch.py'
            ])
        ]),
        launch_arguments={
            'load_transforms': 'false',
            'save_transforms': 'false',
        }.items(),
    )

    tf_map_to_landmark_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_landmark_modules'),
                'launch',
                'tf_map_to_landmark.launch.py'
            ])
        ])
    )

    return (
        LaunchDescription([
            static_transform_pub_launch_include,
            TimerAction(
                actions=[tf_map_to_landmark_launch_include],
                period=2.0,
            ),
            TimerAction(
                actions=[ReadyToTest()],
                period=3.0,
            ),
        ]),
        {
            'static_transform_pub_launch_include': static_transform_pub_launch_include,
            'tf_map_to_landmark': tf_map_to_landmark_launch_include,
        }
    )


class LandmarkTest:

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_landmark')
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node=self.node)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.tf_buffer = tf2_ros.Buffer(node=self.node)
        self.tf_listener = tf2_ros.TransformListener(
            buffer=self.tf_buffer, node=self.node, spin_thread=False
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_label(self):
        label = 'test'
        lm = Landmark(label, 0, node_inf=self.node)
        assert label == lm.label

        new_label = 'new'
        lm.label = new_label
        assert new_label == lm.label

    def test_id(self):
        id_num = 0
        lm = Landmark('test', id_num, node_inf=self.node)
        assert id_num == lm.tag_id

        # shouldn't be able to change tag_id once set
        with pytest.raises(AttributeError):
            lm.tag_id = id_num

    def test_landmark_collection_add_pop(self):
        lc = LandmarkCollection(node_inf=self.node)
        assert lc.is_empty()

        lc.add_landmark('test', 0)
        assert lc.get_landmark(0).tag_id == 0

        assert len(lc) == 1

        popped_lm = lc.pop_landmark(0)
        assert lc.is_empty()

        assert popped_lm.tag_id == 0

    def test_set_and_pub_tfs(self):
        # Need a zeroed tf between map and landmarks first
        tf_map_to_lms = TransformStamped()
        tf_map_to_lms.header.frame_id = 'map'
        tf_map_to_lms.header.stamp = Time(seconds=1, nanoseconds=0).to_msg()
        tf_map_to_lms.child_frame_id = 'landmarks'
        self.tf_broadcaster.sendTransform(tf_map_to_lms)
        self.executor.spin_once(timeout_sec=0.5)

        lc = LandmarkCollection(
            node_inf=self.node,
            ros_on=True,
            tf_buffer=self.tf_buffer,
            tf_listener=self.tf_listener
        )
        lc.add_landmark('test_1', 1)

        tf_1 = Transform()

        lc.get_landmark(1).cam_frame_id = 'landmarks'
        lc.get_landmark(1).tf_wrt_cam = Transform()

        assert self.tf_buffer.can_transform(
            source_frame='map',
            target_frame='landmarks',
            time=Time(),
        )

        lc.get_landmark(1).update_tf(
            parent_old='landmarks',
            parent_new='map'
        )

        lm_1 = lc.get_landmark(1)

        assert lm_1.tf_wrt_cam.transform.translation.x == tf_1.translation.x
        assert lm_1.tf_wrt_cam.transform.translation.y == tf_1.translation.y
        assert lm_1.tf_wrt_cam.transform.translation.z == tf_1.translation.z
        assert lm_1.tf_wrt_cam.transform.rotation.x == tf_1.rotation.x
        assert lm_1.tf_wrt_cam.transform.rotation.y == tf_1.rotation.y
        assert lm_1.tf_wrt_cam.transform.rotation.z == tf_1.rotation.z
        assert lm_1.tf_wrt_cam.transform.rotation.w == tf_1.rotation.w
        assert lm_1.tf_wrt_cam.header.frame_id == 'landmarks'
        assert lm_1.tf_wrt_cam.child_frame_id == 'test_1'

        tf_2 = Transform()
        tf_2.translation.x = 1.0

        lc.add_landmark('test_2', 2)
        lc.get_landmark(2).cam_frame_id = 'landmarks'
        lc.get_landmark(2).tf_wrt_cam = tf_2

        lc.get_landmark(2).update_tf(
            parent_old='landmarks',
            parent_new='map'
        )

        assert lc.get_landmark(2)._tf_set

        lc.pub_tfs(tag_ids=2)

        future_wait_for_tf2: Future = self.tf_buffer.wait_for_transform_async(
            target_frame='map',
            source_frame='test_2',
            time=Time(),
        )

        rclpy.spin_until_future_complete(
            node=self.node,
            executor=self.executor,
            future=future_wait_for_tf2,
            timeout_sec=0.5,
        )

        tf_map_to_test_2 = self.tf_buffer.lookup_transform(
            'map',
            'test_2',
            Time(),
        )

        pos_x = tf_map_to_test_2.transform.translation.x == tf_2.translation.x
        pos_y = tf_map_to_test_2.transform.translation.y == tf_2.translation.y
        pos_z = tf_map_to_test_2.transform.translation.z == tf_2.translation.z

        rot_x = tf_map_to_test_2.transform.rotation.x == tf_2.rotation.x
        rot_y = tf_map_to_test_2.transform.rotation.y == tf_2.rotation.y
        rot_z = tf_map_to_test_2.transform.rotation.z == tf_2.rotation.z
        rot_w = tf_map_to_test_2.transform.rotation.w == tf_2.rotation.w

        assert (pos_x and pos_y and pos_z)
        assert (rot_x and rot_y and rot_z and rot_w)
