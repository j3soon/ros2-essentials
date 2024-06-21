from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation time",
    ),
    DeclareLaunchArgument(
        name="rviz_config_path",
        default_value="",
        description="The path of .rviz file",
    ),
]


# This launch file will only launch rviz2 node.
# It is used when other repos want to launch rviz2 for kobuki. For instance: kobuki_navigation
def generate_launch_description():
    # Launch rviz2.
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_path")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_rviz)

    return ld
