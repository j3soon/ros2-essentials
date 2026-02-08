"""
Pure Navigation Launch File for Stretch3 robot with Isaac Sim.

This launch file starts only the Nav2 navigation stack (no SLAM).
Use this when Isaac Sim provides world -> odom TF (ground truth odometry).

TF Chain (provided by Isaac Sim): world -> odom -> base_link
Laser Scan Topic: /laser_scan
"""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Enable use_sim_time for Isaac Sim",
    )
]


def generate_launch_description():
    """
    Prerequisites:
    - Isaac Sim must provide: world -> odom -> base_link TF chain
    - Isaac Sim must publish: /laser_scan topic
    - Isaac Sim must publish: /odom topic
    """

    # Nav2 parameters file path
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("stretch3_navigation"), "config", "nav2_params.yaml"]
    )

    # Launch Nav2 navigation stack
    launch_nav2_bringup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("nav2_bringup").find("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ]
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_nav2_bringup)

    return ld
