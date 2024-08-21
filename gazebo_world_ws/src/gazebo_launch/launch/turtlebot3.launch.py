from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "launch_gzclient",
        default_value="False",
        description="Launch gzclient, by default is False, which means headless mode",
    ),
]


def generate_launch_description():
    # Get gazebo world file path
    # Available choices:
    # - empty_world
    # - turtlebot3_dqn_stage1
    # - turtlebot3_dqn_stage2
    # - turtlebot3_dqn_stage3
    # - turtlebot3_dqn_stage4
    # - turtlebot3_house
    # - turtlebot3_world
    choice = "turtlebot3_world"
    world_file = PathJoinSubstitution(
        [
            FindPackageShare("turtlebot3_gazebo"),
            "worlds",
            choice + ".world",
        ],
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("gazebo_launch"),
                "launch",
                "gazebo.launch.py",
            ],
        ),
        launch_arguments={
            "world_path": world_file,
            "GAZEBO_MODEL_PATH": get_package_share_directory("turtlebot3_gazebo") + "/models",
            "launch_gzclient": LaunchConfiguration("launch_gzclient"),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_gazebo)

    return ld