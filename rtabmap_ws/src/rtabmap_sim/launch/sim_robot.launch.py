from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declared the world path
    # Ref: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/98a27b20952b11047f454d7ec751f8c742862713
    # File path: turtlebot3_gazebo/worlds/turtlebot3_world.world
    world_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "worlds/turtlebot3_world.world"
    ])
    
    # Gazebo launch file path
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/0f4818cd5d594f044a65e5beaf4a8b296ed00bc6
    # File path: gazebo_ros/launch/gazebo.launch.py
    gazebo_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), 
        "launch", "gazebo.launch.py"
    ])
    
    # Launch Gazebo with the given world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo
    ])