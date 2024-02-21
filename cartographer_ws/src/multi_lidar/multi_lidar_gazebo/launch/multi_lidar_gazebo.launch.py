import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    #####################################
    # Path settings

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

    # Robot description path
    multi_lidar_desp_path = PathJoinSubstitution([
        FindPackageShare("multi_lidar_desp"),
        "launch", "multi_lidar_description.launch.py"
    ])


    #####################################
    # Add launch description

    # Launch the robot with multiple LiDARs
    # - With given robot position arguments
    multi_lidar_desp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_lidar_desp_path),
        launch_arguments={
            'robot_px': '0.5',
            'robot_py': '0.5'
        }.items(),
    )
    
    # Launch Gazebo with the given world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={'world': world_path}.items()
    )

    #####################################
    # Other nodes

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'src/multi_lidar/multi_lidar_gazebo/rviz/multi_lidar.rviz']
    )

    #####################################
    # Launch description
    
    return LaunchDescription([
        gazebo, multi_lidar_desp, rviz
    ])