from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Enable use_sim_time",
    ),
    DeclareLaunchArgument(
        name="launch_rviz",
        default_value="True",
        description="Launch RViz2",
    ),
]


def generate_launch_description():
    # The path of nav2_params file.
    nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_navigation"),
            "config",
            "nav2_params.yaml",
        ]
    )

    # Launch nav2_bringup. ( Use the launch file in "nav2_bringup" )
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
        }.items(),
    )

    # Launch odom_tf_publisher node
    odom_tf_publisher_node = Node(
        package="go2_navigation",
        executable="odom_tf_publisher.py",
        name="odom_tf_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # Launch map -> odom TF publisher
    # NOTE: We ignore the localization part in this demo, so we publish the map -> odom TF directly.
    map_odom_tf_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_tf_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # Launch fake_map_publisher node
    # NOTE: This node publishes a fake map for demonstration purposes.
    fake_map_publisher_node = Node(
        package="go2_navigation",
        executable="fake_map_publisher.py",
        name="fake_map_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # Launch rviz
    rviz_config_file = get_package_share_directory("go2_navigation") + "/rviz/go2_navigation.rviz"
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_nav2_bringup)
    ld.add_action(odom_tf_publisher_node)
    ld.add_action(map_odom_tf_publisher_node)
    ld.add_action(fake_map_publisher_node)
    ld.add_action(rviz_node)

    return ld
