from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "launch_gzclient",
        default_value="True",
        description="Launch gzclient, by default is True, which shows the gazebo GUI",
    ),
    DeclareLaunchArgument(
        "gazebo_world",
        default_value="turtlebot3_world.world",
        description="TurtleBot3 world file",
        choices=[
            "empty_world.world", 
            "turtlebot3_dqn_stage1.world", 
            "turtlebot3_dqn_stage2.world", 
            "turtlebot3_dqn_stage3.world", 
            "turtlebot3_dqn_stage4.world", 
            "turtlebot3_house.world", 
            "turtlebot3_world.world",
        ],
    ),
]


def generate_launch_description():
    world_file = PathJoinSubstitution(
        [
            FindPackageShare("turtlebot3_gazebo"),
            "worlds",
            LaunchConfiguration("gazebo_world"),
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