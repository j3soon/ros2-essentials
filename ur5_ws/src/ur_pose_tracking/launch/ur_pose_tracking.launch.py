from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

ARGUMENTS = [
    DeclareLaunchArgument(
        "config_file",
        default_value="default.yaml",
        choices=["default.yaml"],
        description="The config file to use for the node",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time. (Gazebo)",
    ),
    DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="The ros logging level.",
    ),
]


def generate_launch_description():
    # Config File
    config_file = PathJoinSubstitution([FindPackageShare("ur_pose_tracking"), "config", LaunchConfiguration("config_file")])

    # Set use_sim_time parameter
    use_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    # Spawn Node
    launch_ur_pose_tracking_node = Node(
        package="ur_pose_tracking",
        executable="main.py",
        name="ur_pose_tracking_node",
        output="screen",
        parameters=[config_file],
        arguments=["--ros-args", "--log-level", ["ur_pose_tracking_node:=", LaunchConfiguration("log_level")]],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(use_sim_time)
    ld.add_action(launch_ur_pose_tracking_node)

    return ld
