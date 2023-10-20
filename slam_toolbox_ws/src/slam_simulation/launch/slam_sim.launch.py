from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    
    #####################################
    # Declare Model path
    # Model Path Ref: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble-devel/turtlebot3_gazebo/models
    model_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "models"
    ])
    
    waffle_model_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "models/turtlebot3_waffle/model.sdf"
    ])
    
    # Gazebo launch file path
    # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/launch/gazebo.launch.py
    gazebo_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), 
        "launch", "gazebo.launch.py"
    ])
    
    # RViz path
    rviz_path = PathJoinSubstitution([
        FindPackageShare("slam_simulation"), 
        "rviz", "slam_sim.rviz"
    ])
    
    # SLAM toolbox config file
    # Ref: https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml
    slam_param_path = PathJoinSubstitution([
        FindPackageShare("slam_simulation"), 
        "config", "mapper_online_async.yaml"
    ])
    
    #####################################
    # Arguments
    slam_param = LaunchConfiguration("slam_param")
    slam_params_arg = DeclareLaunchArgument(
        'slam_param', default_value = slam_param_path
    )
    
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
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path)
    )
    
    # TF Ref: 
    # https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html#the-proper-way-to-publish-static-transforms
    # Static TF : base_footprint -> base_link
    foot_to_link_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "foot_to_link",
        arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
        parameters=[{'use_sim_time': True}]
    )
    
    # Static TF : base_link -> base_scan
    link_to_scan_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "link_to_scan",
        arguments = ["0", "0", "0", "0", "0", "0", "base_link", "base_scan"],
        parameters=[{'use_sim_time': True}]
    )
    
    # Static TF : base_link -> imu
    link_to_imu_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "link_to_imu",
        arguments = ["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        parameters=[{'use_sim_time': True}]
    )
    
    # SLAM toolbox node
    # Ref: https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py
    slam = Node(
        parameters=[
          slam_param,
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox'
    )
    
    #####################################
    # Launch description
    return LaunchDescription([
        
        # Arguments
        slam_params_arg,
        
        # Simulation and Monitor
        gazebo, spawn_entity, rviz,
        
        # Static TF
        foot_to_link_tf2, link_to_scan_tf2, link_to_imu_tf2,
        
        # Mapping
        slam
    ])