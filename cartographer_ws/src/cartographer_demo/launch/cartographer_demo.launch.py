from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    #####################################
    # Path and argument settings

    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare("cartographer_demo"), 'config'
    ])
    cartographer_config_basename = 'cartographer.lua'

    is_sim = LaunchConfiguration('is_sim')
    is_sim_arg = DeclareLaunchArgument('is_sim', default_value='True')

    resolution = LaunchConfiguration('resolution')
    resolution_arg = DeclareLaunchArgument('resolution', default_value='0.05')
    
    #####################################
    # Nodes

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': is_sim}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', cartographer_config_basename]
    )
    
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': is_sim}],
        arguments=['-resolution', resolution, '-publish_period_sec', "1.0"]
    )
    
    #####################################
    # Launch description

    return LaunchDescription([

        is_sim_arg, resolution_arg,
        
        cartographer, occupancy_grid
    ])