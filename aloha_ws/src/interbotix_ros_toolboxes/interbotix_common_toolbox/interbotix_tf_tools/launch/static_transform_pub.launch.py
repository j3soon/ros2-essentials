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
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'load_transforms',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'whether or not the static_trans_pub node should publish any poses stored in a '
                'static_transforms config file at startup.'
            ),
        ),
        DeclareLaunchArgument(
            'save_transforms',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'whether or not the static_trans_pub node should save all TFs to a '
                'static_transforms config file.'
            ),
        ),
        DeclareLaunchArgument(
            'transform_filename',
            default_value=TextSubstitution(text='static_transforms.yaml'),
            description='the filename to which the transforms should be saved.',
        ),
        DeclareLaunchArgument(
            'transform_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_tf_tools'),
                'config',
                LaunchConfiguration('transform_filename')
            ]),
            description='the absolute filepath to which the transforms should be saved.',
        ),
    ])

    static_trans_pub_node = Node(
        name='static_trans_pub',
        package='interbotix_tf_tools',
        executable='static_trans_pub.py',
        parameters=[{
            'load_transforms': LaunchConfiguration('load_transforms'),
            'save_transforms': LaunchConfiguration('save_transforms'),
            'transform_filepath': LaunchConfiguration('transform_filepath'),
        }],
    )

    launch_description.add_action(static_trans_pub_node)

    return launch_description
