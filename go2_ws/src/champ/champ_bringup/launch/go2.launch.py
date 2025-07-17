import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation time",
    ),
]


def generate_launch_description():
    # Get the config paths
    go2_description_path = get_package_share_directory("go2_description")
    champ_config_path = get_package_share_directory("champ_config")
    urdf_path = PathJoinSubstitution(
        [
            go2_description_path,
            "xacro",
            "robot.xacro",
        ]
    )
    joint_config_path = PathJoinSubstitution(
        [
            champ_config_path,
            "config/joints",
            "joints.yaml",
        ]
    )
    links_config_path = PathJoinSubstitution(
        [
            champ_config_path,
            "config/links",
            "links.yaml",
        ]
    )
    gait_config_path = PathJoinSubstitution(
        [
            champ_config_path,
            "config/gait",
            "gait.yaml",
        ]
    )

    # Bring up the champ controller for Go2
    launch_champ_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("champ_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "description_path": urdf_path,
            "joints_map_path": joint_config_path,
            "links_map_path": links_config_path,
            "gait_config_path": gait_config_path,
            "robot_name": "go2",
            "gazebo": "false",
            "publish_joint_control": "false",
            "publish_joint_states": "true",
            "publish_foot_contacts": "false",
            "publish_odom_tf": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_champ_bringup)

    return ld
