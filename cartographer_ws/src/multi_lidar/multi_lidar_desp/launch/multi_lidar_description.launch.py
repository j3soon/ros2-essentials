import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the robot position argument
    robot_px = LaunchConfiguration('robot_px')
    robot_px_arg = DeclareLaunchArgument('robot_px', default_value='0.5')

    robot_py = LaunchConfiguration('robot_py')
    robot_py_arg = DeclareLaunchArgument('robot_py', default_value='0.5')

    robot_pz = LaunchConfiguration('robot_pz')
    robot_pz_arg = DeclareLaunchArgument('robot_pz', default_value='0.0')

    # Define the path to the model
    waffle_model_path = PathJoinSubstitution([
        FindPackageShare("multi_lidar_desp"), 
        "models/turtlebot3_waffle/model.sdf"
    ])
    
    # Add robot description
    # Ref: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/98a27b20952b11047f454d7ec751f8c742862713
    # File path: turtlebot3_gazebo/launch/robot_state_publisher.launch.py
    urdf_path = os.path.join(
        get_package_share_directory('multi_lidar_desp'),
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

    # Spawn the robot in Gazebo
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/launch/spawn_entity_demo.launch.py
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-file', waffle_model_path,
            '-x', robot_px,
            '-y', robot_py,
            '-z', robot_pz
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Argument
        robot_pz_arg, robot_px_arg, robot_py_arg,

        # Nodes
        robot_description, spawn_entity
    ])