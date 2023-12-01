from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        name="is_sim",
        default_value="False",
        description="Use gazebo simulation or use real robot",
    ),
]


def generate_launch_description():
    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_gazebo"),
                "launch",
                "gazebo.launch.py",
            ],
        ),
        launch_arguments={
            "launch_rviz": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("is_sim")),
    )

    # Launch real robot
    launch_real_robot = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_node"),
                "launch",
                "kobuki_node-launch.py",
            ],
        ),
        condition=UnlessCondition(LaunchConfiguration("is_sim")),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_gazebo)
    ld.add_action(launch_real_robot)

    return ld
