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
    TextSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    tf_rebroadcaster_container = ComposableNodeContainer(
        name='tf_rebroadcaster_container',
        namespace=LaunchConfiguration('namespace_to'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='interbotix_tf_tools',
                plugin='interbotix_tf_tools::TFRebroadcaster',
                name='tf_rebroadcaster',
                namespace=LaunchConfiguration('namespace_to'),
                parameters=[
                    {
                        'filepath_config': LaunchConfiguration('tf_rebroadcaster_config'),
                        'topic_from': LaunchConfiguration('topic_from'),
                    },
                ],
                remappings=[('/tf', [LaunchConfiguration('namespace_to'), '/tf'])],
            )
        ]
    )

    return [
        tf_rebroadcaster_container,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'tf_rebroadcaster_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_tf_tools'),
                'config',
                'tf_rebroadcaster.yaml'
            ]),
            description='filepath to the tf_rebroadcaster configuration file.',
        ),
        DeclareLaunchArgument(
            'topic_from',
            description='topic from which the TFs will be retrieved.',
        ),
        DeclareLaunchArgument(
            'namespace_to',
            default_value=TextSubstitution(text=''),
            description='namespace under which the TF messages will be re-broadcasted.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
