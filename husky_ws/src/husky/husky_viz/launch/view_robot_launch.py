from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# This launch file will only launch rviz2 node.
# It is used when other repos want to launch rviz2 for Husky. For instance: husky_navigation
def generate_launch_description():

    # The path of .rviz file.
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("husky_viz"), "rviz", "slam.rviz"]
    )

    # Launch rviz2.
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
