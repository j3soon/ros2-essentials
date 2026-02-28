from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the rviz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('stretch3_navigation'),
        'rviz',
        'stretch3.rviz'
    )

    # Launch RViz with the stretch3 configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
