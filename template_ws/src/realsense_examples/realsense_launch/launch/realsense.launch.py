from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    realsense_config = PathJoinSubstitution(
        [FindPackageShare("realsense_launch"), "config", "realsense.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument("config_file", default_value=realsense_config),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
    ]

    # launch RealSense
    launch_realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # Launch rviz
    rviz_config_file = get_package_share_directory("realsense_launch") + "/rviz/realsense.rviz"
    launch_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    ld = LaunchDescription(declared_arguments)
    ld.add_action(launch_realsense)
    ld.add_action(launch_rviz)

    return ld