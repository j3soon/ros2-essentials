from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    #####################################
    # Get argument by launch configuration
    # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html

    using_odom = LaunchConfiguration('using_odom')
    using_odom_arg = DeclareLaunchArgument('using_odom', default_value='False')

    #####################################
    # Path settings
    # Using turtlebot3 cartographer demo launch
    # Ref: https://github.com/ROBOTIS-GIT/turtlebot3/tree/a7dd05ae176f3f3778b0a36f7065dc9655b050e3/turtlebot3_cartographer/launch/cartographer.launch.py

    cartographer_demo_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_cartographer"),
        "launch", "cartographer.launch.py"
    ])

    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare("vlp_cartographer"),
        "config"
    ])
    
    # Some minor settings such as frame name is different,
    # we should use our own config file
    cartographer_demo_basename = "cartographer_demo.lua"
    cartographer_with_odom_basename = "cartographer_with_odom.lua"

    #####################################
    # Add launch description

    cartographer_demo = IncludeLaunchDescription(
        cartographer_demo_path,
        launch_arguments={
            'cartographer_config_dir': cartographer_config_dir,
            'configuration_basename': cartographer_demo_basename
        }.items(),
        condition=UnlessCondition(using_odom)
    )

    cartographer_with_odom = IncludeLaunchDescription(
        cartographer_demo_path,
        launch_arguments={
            'cartographer_config_dir': cartographer_config_dir,
            'configuration_basename': cartographer_with_odom_basename
        }.items(),
        condition=IfCondition(using_odom)
    )
    
    #####################################
    # Launch description
    return LaunchDescription([

        # --- Arguments ---
        using_odom_arg,

        # --- Nodes ---
        cartographer_demo, cartographer_with_odom
    ])