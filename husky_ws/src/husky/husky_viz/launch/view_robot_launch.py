from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("husky_viz"), "rviz", "test.rviz"]
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription(
        [
            node_rviz,
        ]
    )
