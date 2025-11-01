# Copyright 2023 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import Node


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    str = str.replace(' ', '')
    arr = str.split(',')

    return arr

def launch_setup(context, *args, **kwargs):
    
    # URDF/xacro file to be loaded by the Robot State Publisher node
    multi_zed_xacro_path = os.path.join(
    get_package_share_directory('zed_multi_camera'),
    'urdf',
    'zed_multi.urdf.xacro')

    robot_description = Command(['xacro', ' ', multi_zed_xacro_path]).perform(context)

    # Robot State Publisher node
    # this will publish the static reference link for a multi-camera configuration
    # and all the joints. See 'urdf/zed_dual.urdf.xacro' as an example
    multi_rsp_node = Node(
        package='robot_state_publisher',
        namespace='zed_multi',
        executable='robot_state_publisher',
        name='zed_multi_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    names = LaunchConfiguration('cam_names')
    models = LaunchConfiguration('cam_models')
    serials = LaunchConfiguration('cam_serials')
    disable_tf = LaunchConfiguration('disable_tf')

    names_arr = parse_array_param(names.perform(context))
    models_arr = parse_array_param(models.perform(context))
    serials_arr = parse_array_param(serials.perform(context))
    disable_tf_val = disable_tf.perform(context)

    num_cams = len(names_arr)

    if (num_cams != len(models_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text='The size of the `models` param array must be equal to the size of `names`'))
        ]

    if (num_cams != len(serials_arr)):
        return [
            LogInfo(msg=TextSubstitution(
                text='The size of the `serials` param array must be equal to the size of `names`'))
        ]

    # Add the robot_state_publisher node to the list of nodes to be started
    actions = [multi_rsp_node]

    # Set the first camera idx
    cam_idx = 0

    for name in names_arr:
        model = models_arr[cam_idx]
        serial = serials_arr[cam_idx]
        pose = '['

        info = '* Starting a ZED ROS2 node for camera ' + name + \
            '(' + model + '/' + serial + ')'

        actions.append(LogInfo(msg=TextSubstitution(text=info)))

        # Only the first camera send odom and map TF
        publish_tf = 'false'
        if (cam_idx == 0):
            if (disable_tf_val == 'False' or disable_tf_val == 'false'):
                publish_tf = 'true'

        # A different node name is required by the Diagnostic Updated
        node_name = 'zed_node_' + str(cam_idx)

        # Add the node
        # ZED Wrapper launch file
        zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_name': name,
                'camera_model': model,
                'serial_number': serial,
                'publish_tf': publish_tf,
                'publish_map_tf': publish_tf,
                'node_name': node_name
            }.items()
        )

        actions.append(zed_wrapper_launch)

        cam_idx += 1

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'cam_names',
                description='An array containing the names of the cameras, e.g. [zed_front,zed_back]'),
            DeclareLaunchArgument(
                'cam_models',
                description='An array containing the names of the cameras, e.g. [zed2i,zed2]'),
            DeclareLaunchArgument(
                'cam_serials',
                description='An array containing the serial numbers of the cameras, e.g. [35199186,23154724]'),
            DeclareLaunchArgument(
                'disable_tf',
                default_value='False',
                description='If `True` disable TF broadcasting for all the cameras in order to fuse visual odometry information externally.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
