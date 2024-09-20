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
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    filter_ns_launch_arg = LaunchConfiguration('filter_ns')
    filter_params_launch_arg = LaunchConfiguration('filter_params')
    use_pointcloud_tuner_gui = LaunchConfiguration('use_pointcloud_tuner_gui')
    enable_pipeline_launch_arg = LaunchConfiguration('enable_pipeline')
    cloud_topic_launch_arg = LaunchConfiguration('cloud_topic')

    pointcloud_pipeline_node = Node(
        package='interbotix_perception_pipelines',
        executable='pointcloud_pipeline',
        name='pointcloud_pipeline',
        namespace=filter_ns_launch_arg,
        parameters=[
            filter_params_launch_arg,
            {
                'cloud_topic': cloud_topic_launch_arg,
                'enable_pipeline': enable_pipeline_launch_arg,
            }
        ],
    )

    pointcloud_tuner_gui_node = Node(
        package='interbotix_perception_modules',
        executable='pointcloud_tuner_gui',
        namespace=filter_ns_launch_arg,
        parameters=[
            filter_params_launch_arg,
            {
                'filter_params': filter_params_launch_arg,
            }
        ],
        arguments=[
            '--filter_ns', filter_ns_launch_arg.perform(context),
        ],
        condition=IfCondition(use_pointcloud_tuner_gui),
    )

    return [
        pointcloud_pipeline_node,
        pointcloud_tuner_gui_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'filter_ns',
            default_value='pc_filter',
            description='namespace where the pointcloud related nodes and parameters are located.',
        ),
        DeclareLaunchArgument(
            'filter_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'config',
                'filter_params.yaml'
            ]),
            description='filepath of the parameters used to tune the perception pipeline filters.',
        ),
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui',
            default_value='false',
            choices=('true', 'false'),
            description='whether to show a GUI that a user can use to tune filter parameters.',
        ),
        DeclareLaunchArgument(
            'enable_pipeline',
            default_value='false',
            description=(
                'whether to enable the perception pipeline filters to run continuously; to save '
                'computer processing power, this should be set to `false` unless you are actively '
                'trying to tune the filter parameters; if `false`, the pipeline will only run if '
                'the `get_cluster_positions` ROS service is called.'
            ),
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='camera/depth/color/points',
            description='the absolute ROS topic name to subscribe to raw pointcloud data.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
