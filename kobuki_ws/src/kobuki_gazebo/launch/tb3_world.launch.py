from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get gazebo world file path
    world_file = PathJoinSubstitution(
        [
            FindPackageShare("kobuki_gazebo"),
            "worlds",
            "turtlebot3_stage_2.world",
        ],
    )

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
            "world_path": world_file,
            "launch_rviz": "True",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(launch_gazebo)

    return ld
