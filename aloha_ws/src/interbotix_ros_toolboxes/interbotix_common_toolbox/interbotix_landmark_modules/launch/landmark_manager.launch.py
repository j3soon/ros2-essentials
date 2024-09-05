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

    landmark_config_launch_arg = LaunchConfiguration('landmark_config')
    fixed_frame_launch_arg = LaunchConfiguration('fixed_frame')
    observation_frame_launch_arg = LaunchConfiguration('observation_frame')
    standalone_tags_launch_arg = LaunchConfiguration('standalone_tags')

    landmark_manager_node = Node(
        package='interbotix_landmark_modules',
        executable='landmark_manager',
        name='landmark_manager',
        parameters=[
            landmark_config_launch_arg,
            standalone_tags_launch_arg,
            {
                'fixed_frame': fixed_frame_launch_arg,
                'observation_frame': observation_frame_launch_arg,
            }
        ],
        output={'both': 'screen'},
    )

    return [
        landmark_manager_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'landmark_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_landmark_modules'),
                'landmarks',
                'landmarks.yaml'
            ]),
            description='absolute filepath to the landmarks configuration file.',
        ),
        DeclareLaunchArgument(
            'fixed_frame',
            default_value='landmarks',
            description='fixed frame that all landmarks should be a chlid of in their transforms.',
        ),
        DeclareLaunchArgument(
            'observation_frame',
            default_value='camera_color_optical_frame',
            description='frame from which the landmarks will be observed from.',
        ),
        DeclareLaunchArgument(
            'standalone_tags',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'config',
                'tags.yaml'
            ]),
            description=(
              'absolute filepath to AprilTags configuration file that the landmarks are '
              'represented by.'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
