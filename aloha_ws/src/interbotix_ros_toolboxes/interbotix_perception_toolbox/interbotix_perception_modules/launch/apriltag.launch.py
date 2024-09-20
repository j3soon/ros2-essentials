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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):

    tags_config_launch_arg = LaunchConfiguration('tags_config')
    camera_frame_launch_arg = LaunchConfiguration('camera_frame')
    apriltag_ns_launch_arg = LaunchConfiguration('apriltag_ns')
    camera_color_topic_launch_arg = LaunchConfiguration('camera_color_topic')
    camera_info_topic_launch_arg = LaunchConfiguration('camera_info_topic')

    apriltag_ros_single_image_detector_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_single_image_detector_node',
        name='ar_tracker',
        namespace=apriltag_ns_launch_arg.perform(context),
        parameters=[
            tags_config_launch_arg.perform(context),
        ],
        arguments=[
            apriltag_ns_launch_arg.perform(context)
        ],
    )

    picture_snapper_node = Node(
        package='interbotix_perception_modules',
        executable='picture_snapper',
        name='picture_snapper',
        namespace=apriltag_ns_launch_arg.perform(context),
        parameters=[
            {
                'camera_color_topic': camera_color_topic_launch_arg,
                'camera_info_topic': camera_info_topic_launch_arg,
                'camera_frame': camera_frame_launch_arg,
            }
        ],
        arguments=[
            apriltag_ns_launch_arg.perform(context)
        ],
    )

    return [
        apriltag_ros_single_image_detector_node,
        picture_snapper_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'tags_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'config',
                'tags.yaml'
            ]),
            description='filepath to the AprilTag tags configuration file.',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_color_optical_frame',
            description='the camera frame in which the AprilTag will be detected.',
        ),
        DeclareLaunchArgument(
            'apriltag_ns',
            default_value='apriltag',
            description='namespace where the AprilTag related nodes and parameters are located.',
        ),
        DeclareLaunchArgument(
            'camera_color_topic',
            default_value='camera/color/image_raw',
            description='the absolute ROS topic name to subscribe to color images.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='camera/color/camera_info',
            description='the absolute ROS topic name to subscribe to the camera color info.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
