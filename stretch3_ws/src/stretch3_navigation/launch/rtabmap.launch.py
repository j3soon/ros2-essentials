"""
Rtabmap SLAM Launch File for Stretch3 robot with Isaac Sim.

This launch file starts:
- Rtabmap for 2D/3D SLAM (publishes map -> odom)
- Nav2 navigation stack for autonomous navigation

TF Chain: map -> odom -> base_link (odom->base_link from Isaac Sim)
Laser Scan Topic: /laser_scan
Camera Topics:
    - RGB Image: /rgb
    - Depth Image: /depth
    - Camera Info: /camera_info
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Enable use_sim_time for Isaac Sim",
    )
]

def generate_launch_description():

    # Load the rtabmap settings file
    rtabmap_settings = PathJoinSubstitution([
        FindPackageShare("stretch3_navigation"), 
        "config/rtabmap.yaml"
    ])

    # Setup the remaps for camera topics
    remappings = [
        ('rgb/image', '/rgb'),
        ('rgb/camera_info', '/camera_info'),
        ('depth/image', '/depth'),
        ('scan', '/laser_scan')
    ]
    
    # Rtabmap SLAM node
    rtabmap_slam = Node(
        package = 'rtabmap_slam', 
        executable = 'rtabmap',
        parameters = [rtabmap_settings],
        remappings = remappings,
        arguments = ['-d']
    )
    
    # Rtabmap visualization node
    rtabmapviz = Node(
        package = 'rtabmap_viz',
        executable = 'rtabmap_viz',
        parameters = [rtabmap_settings],
        remappings = remappings
    )

    # Static transform: world -> map (connects Isaac Sim's world to Rtabmap's map)
    static_tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
    ld.add_action(rtabmap_slam)
    ld.add_action(rtabmapviz)
    ld.add_action(static_tf_world_to_map)
    ld.add_action(launch_nav2_bringup)

    return ld