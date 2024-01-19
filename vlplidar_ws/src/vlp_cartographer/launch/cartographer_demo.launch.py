from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    #####################################
    # Path settings
    # Using turtlebot3 cartographer demo launch
    # Ref: https://github.com/ROBOTIS-GIT/turtlebot3/tree/a7dd05ae176f3f3778b0a36f7065dc9655b050e3
    # File Path: turtlebot3_cartographer/launch/cartographer.launch.py

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
    cartographer_basename = "cartographer_demo.lua"

    #####################################
    # Add launch description

    cartographer_demo = IncludeLaunchDescription(
        cartographer_demo_path,
        launch_arguments={
            'cartographer_config_dir': cartographer_config_dir,
            'configuration_basename': cartographer_basename
        }.items(),
    )
    
    #####################################
    # Launch description
    return LaunchDescription([
        cartographer_demo
    ])