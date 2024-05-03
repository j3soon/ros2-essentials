from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get gazebo world file path
    world_file = PathJoinSubstitution(
        [
            FindPackageShare("aws_robomaker_small_house_world"),
            "worlds",
            "small_house.world",
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
            "GAZEBO_MODEL_PATH": get_package_share_directory("aws_robomaker_small_house_world") + "/models",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(launch_gazebo)

    return ld