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
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation time",
    ),
    DeclareLaunchArgument(
        "world_path",
        default_value="",
        description="The world path, by default is empty.world",
    ),
    DeclareLaunchArgument(
        "launch_rviz",
        default_value="False",
        description="Launch rviz2, by default is False",
    ),
    DeclareLaunchArgument(
        "GAZEBO_MODEL_PATH",
        default_value="",
        description="The path to the gazebo models",
    ),
    DeclareLaunchArgument(
        "robot_init_x",
        default_value="0.0",
        description="The initial x position of the robot",
    ),
    DeclareLaunchArgument(
        "robot_init_y",
        default_value="0.0",
        description="The initial y position of the robot",
    ),
    DeclareLaunchArgument(
        "robot_init_z",
        default_value="0.0",
        description="The initial z position of the robot",
    ),
    DeclareLaunchArgument(
        "robot_init_yaw",
        default_value="0.0",
        description="The initial yaw of the robot",
    ),
]


def generate_launch_description():
    # Set GAZEBO_MODEL_PATH
    gz_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            LaunchConfiguration("GAZEBO_MODEL_PATH"),
            ":",
            "/usr/share/gazebo-11/models/",
            ":",
            str(
                Path(get_package_share_directory("kobuki_description")).parent.resolve()
            ),
            ":",
            get_package_share_directory("kobuki_gazebo") + "/models",
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
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
        }.items(),
    )

    # The path of .rviz file.
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("kobuki_rviz"), "rviz", "gazebo.rviz"]
    )

    # Launch rviz2.
    launch_rviz = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_rviz"),
                "launch",
                "open_rviz_launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz_config_path": rviz_config_path,
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # Launch Kobuki's description.
    launch_kobuki_description = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_description"),
                "launch",
                "robot_description.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Launch Kobuki's control
    launch_kobuki_control = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_control"),
                "launch",
                "control.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_kobuki",
        arguments=[
            "-entity", "kobuki",
            "-topic", "/robot_description",
            "-x", LaunchConfiguration("robot_init_x"),
            "-y", LaunchConfiguration("robot_init_y"),
            "-z", LaunchConfiguration("robot_init_z"),
            "-Y", LaunchConfiguration("robot_init_yaw"),
        ],
        output="screen",
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(launch_gazebo)
    ld.add_action(launch_kobuki_description)
    ld.add_action(launch_kobuki_control)
    ld.add_action(spawn_robot)
    ld.add_action(launch_rviz)

    return ld
