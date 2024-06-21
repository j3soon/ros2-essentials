from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Enable use_sim_time",
    )
]


def generate_launch_description():
    localization_config_path = PathJoinSubstitution(
        [FindPackageShare("kobuki_control"), "config", "localization.yaml"],
    )

    launch_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            localization_config_path,
        ],
    )

    imu_config_path = PathJoinSubstitution(
        [FindPackageShare("kobuki_control"), "config", "imu_filter.yaml"],
    )

    launch_imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            imu_config_path,
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_ekf_node)
    ld.add_action(launch_imu_filter)

    return ld