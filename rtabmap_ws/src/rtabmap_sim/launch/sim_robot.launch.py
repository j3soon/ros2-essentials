import os

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    waffle_model_path = PathJoinSubstitution([
        FindPackageShare("rtabmap_sim"), 
        "models/turtlebot3_waffle/model.sdf"
    ])

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
    
    # Spawn the robot in Gazebo
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/launch/spawn_entity_demo.launch.py
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', waffle_model_path,
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.0'
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # Add robot description
    # Ref: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/98a27b20952b11047f454d7ec751f8c742862713
    # File path: turtlebot3_gazebo/launch/robot_state_publisher.launch.py
    urdf_path = os.path.join(
        get_package_share_directory('rtabmap_sim'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }],
    )

    return LaunchDescription([
        gazebo, robot_description, spawn_entity
    ])