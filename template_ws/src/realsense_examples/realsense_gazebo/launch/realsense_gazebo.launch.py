import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    #####################################
    # Path settings

    # Declared the world path
    # Ref: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/98a27b20952b11047f454d7ec751f8c742862713/turtlebot3_gazebo/worlds/turtlebot3_world.world
    world_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "worlds/turtlebot3_world.world"
    ])

    # Gazebo launch file path
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/0f4818cd5d594f044a65e5beaf4a8b296ed00bc6/gazebo_ros/launch/gazebo.launch.py
    gazebo_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), 
        "launch", "gazebo.launch.py"
    ])

    # Robot description path
    realsense_description_path = PathJoinSubstitution([
        FindPackageShare("realsense_description"),
        "launch", "realsense_description.launch.py"
    ])

    # Ensure Gazebo classic can resolve model:// URIs used inside model.sdf
    # (e.g. model://turtlebot3_burger/meshes/d435.dae)
    realsense_models_dir = os.path.join(
        get_package_share_directory('realsense_description'),
        'models'
    )
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = realsense_models_dir + os.pathsep + gazebo_model_path
    else:
        gazebo_model_path = realsense_models_dir

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path,
    )


    #####################################
    # Add launch description

    # Launch the robot with realsense
    # - With given robot position arguments
    realsense_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_description_path),
        launch_arguments={
            'robot_px': '-2.0',
            'robot_py': '-0.5',
            'robot_pz': '0.01'
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
    rviz_config = PathJoinSubstitution([FindPackageShare("realsense_gazebo"), "rviz", "realsense.rviz"])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config]
    )

    #####################################
    # Launch description
    
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo, rviz, realsense_description
    ])