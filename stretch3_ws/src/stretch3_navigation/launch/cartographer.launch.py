"""
Cartographer SLAM Launch File for Stretch3 robot with Isaac Sim.

This launch file starts:
- Google Cartographer for 2D SLAM (publishes map -> odom)
- Nav2 navigation stack for autonomous navigation

TF Chain: map -> odom -> base_link (odom->base_link from Isaac Sim)
Laser Scan Topic: /laser_scan
"""

from launch import LaunchDescription
from launch_ros.actions import Node
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
    Launch file for Stretch3 navigation with Cartographer SLAM.
    
    Prerequisites:
    - Isaac Sim must provide: odom -> base_link TF
    - Isaac Sim must publish: /laser_scan, /odom topics
    """

    # Cartographer configuration
    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare("stretch3_navigation"), "config"]
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('scan', '/laser_scan'),
            ('odom', '/odom'),
        ],
    )

    # Static transform: world -> map (connects Isaac Sim's world to Cartographer's map)
    static_tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Cartographer occupancy grid node (publishes /map)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
    )

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
    ld.add_action(static_tf_world_to_map)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(launch_nav2_bringup)

    return ld
