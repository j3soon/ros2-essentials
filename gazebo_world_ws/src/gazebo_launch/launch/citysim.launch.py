from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get gazebo world file path
    world_file = PathJoinSubstitution(
        [
            FindPackageShare("citysim"),
            "worlds",
            "simple_city.world",
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
            "GAZEBO_MODEL_PATH": get_package_share_directory("citysim") + "/models",
            "GAZEBO_RESOURCE_PATH": get_package_share_directory("citysim"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(launch_gazebo)

    return ld