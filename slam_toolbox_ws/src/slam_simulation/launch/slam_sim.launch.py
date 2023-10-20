from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    #####################################
    # Declare Model path
    model_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "models"
    ])
    
    waffle_model_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "models/turtlebot3_waffle/model.sdf"
    ])
    
    # Gazebo launch file path
    gazebo_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), 
        "launch", "gazebo.launch.py"
    ])
    
    # RViz path
    rviz_path = PathJoinSubstitution([
        FindPackageShare("slam_simulation"), 
        "rviz", "slam_sim.rviz"
    ])
    
    #####################################
    # Nodes
    rviz = Node(
        package = "rviz2",
        namespace = "",
        executable = "rviz2",
        name = "rviz2",
        arguments = ["-d", [rviz_path]],
        parameters=[{'use_sim_time': True}]
    )
    
    # Robot Model in Gazebo
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/launch/spawn_entity_demo.launch.py
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_path,
            '-file', waffle_model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch Gazebo
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/launch/gazebo.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path)
    )
    
    #####################################
    # Launch description
    return LaunchDescription([
        
        # Simulation and Monitor
        gazebo, spawn_entity, rviz
    ])