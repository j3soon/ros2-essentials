from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the Model path
    model_path = PathJoinSubstitution([
        FindPackageShare("rtabmap_sim"), 
        "models"
    ])
    
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
            '-entity', model_path,
            '-file', waffle_model_path,
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.0'
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo, spawn_entity
    ])