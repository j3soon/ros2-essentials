from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        "launch_gzclient",
        default_value="False",
        description="Launch gzclient, by default is False, which means headless mode",
    ),
    DeclareLaunchArgument(
        "world_path",
        default_value="",
        description="The world path, by default is empty.world",
    ),
    DeclareLaunchArgument(
        "GAZEBO_MODEL_PATH",
        default_value="",
        description="The path to the gazebo models",
    ),
    DeclareLaunchArgument(
        "GAZEBO_RESOURCE_PATH",
        default_value="",
        description="The path to the gazebo resources",
    ),
]


def generate_launch_description():
    # Set GAZEBO_MODEL_PATH
    gz_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            LaunchConfiguration("GAZEBO_MODEL_PATH"),
            ":",
            "/usr/share/gazebo-11/models/",
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    # Set GAZEBO_RESOURCE_PATH
    gz_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=[
            LaunchConfiguration("GAZEBO_RESOURCE_PATH"),
            ":",
            "/usr/share/gazebo-11/",
            ":",
            EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
        ]
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world_path"),
            "gui": LaunchConfiguration("launch_gzclient"),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_model_path)
    ld.add_action(gz_resource_path)
    ld.add_action(launch_gazebo)

    return ld