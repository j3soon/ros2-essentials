from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    #####################################
    # Get argument by launch configuration
    # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html

    ns = LaunchConfiguration('ns')
    ns_arg = DeclareLaunchArgument('ns', default_value='')

    is_sim = LaunchConfiguration('is_sim')
    is_sim_arg = DeclareLaunchArgument('is_sim', default_value='False')

    publish_static_tf = LaunchConfiguration('publish_static_tf')
    publish_static_tf_arg = DeclareLaunchArgument('publish_static_tf', default_value='True')

    robot_frame = LaunchConfiguration('robot_frame')
    robot_frame_arg = DeclareLaunchArgument('robot_frame', default_value='base_footprint')

    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='velodyne')

    lidar_pose_x = LaunchConfiguration('tf_x')
    lidar_pose_x_arg = DeclareLaunchArgument('tf_x', default_value='0')

    lidar_pose_y = LaunchConfiguration('tf_y')
    lidar_pose_y_arg = DeclareLaunchArgument('tf_y', default_value='0')

    lidar_pose_z = LaunchConfiguration('tf_z')
    lidar_pose_z_arg = DeclareLaunchArgument('tf_z', default_value='0')

    lidar_pose_yaw = LaunchConfiguration('tf_yaw')
    lidar_pose_yaw_arg = DeclareLaunchArgument('tf_yaw', default_value='0')

    #####################################
    # Path settings

    # 1. Underlayer driver launch file path
    # Ref: https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3/velodyne_driver/launch/velodyne_driver_node-VLP16-launch.py
    vlp_driver_path = PathJoinSubstitution([
        FindPackageShare("velodyne_driver"),
        "launch", "velodyne_driver_node-VLP16-launch.py"
    ])

    # 2. Transformation node (raw data -> point cloud)
    # Ref: https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3/velodyne_pointcloud/launch/velodyne_transform_node-VLP16-launch.py
    transformation_path = PathJoinSubstitution([
        FindPackageShare("velodyne_pointcloud"),
        "launch", "velodyne_transform_node-VLP16-launch.py"
    ])

    # 3. Laser scan node (point cloud -> laser scan)
    # Ref: https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3/velodyne_laserscan/launch/velodyne_laserscan_node-launch.py
    laserscan_node_path = PathJoinSubstitution([
        FindPackageShare("velodyne_laserscan"),
        "launch", "velodyne_laserscan_node-launch.py"
    ])

    #####################################
    # Add launch description

    vlp_driver = IncludeLaunchDescription(
        vlp_driver_path, 
        condition=UnlessCondition(
            is_sim
        )
    )

    transformation = IncludeLaunchDescription(
        transformation_path, 
        condition=UnlessCondition(
            is_sim
        )
    )

    laserscan_node = IncludeLaunchDescription(laserscan_node_path)

    #####################################
    # Add to namespace
    # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html
    # 

    driver_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(ns),
            vlp_driver,       # LiDAR underlayer driver
            transformation,   # Transformation node (raw -> point cloud)
            laserscan_node    # Laser scan node (point cloud -> laser scan)
        ]
    )

    # Static transform publisher from base_footprint to velodyne
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            # x, y, z, qx, qy, qz, qw
            lidar_pose_x, lidar_pose_y, lidar_pose_z, 
            lidar_pose_yaw, '0', '0',
            # parent_frame, child_frame
            robot_frame, lidar_frame
        ],
        parameters=[{'use_sim_time': is_sim}],
        output='screen',
        condition=IfCondition(publish_static_tf)
    )
    
    #####################################
    # Launch description
    return LaunchDescription([

        # --- Arguments ---
        # Namespace, Simulation, Static TF Publisher
        ns_arg, is_sim_arg, publish_static_tf_arg,
        # LiDAR frame, Robot frame
        lidar_frame_arg, robot_frame_arg,
        # LiDAR pose (x, y, z, yaw)
        lidar_pose_x_arg, lidar_pose_y_arg, lidar_pose_z_arg, lidar_pose_yaw_arg,

        # --- Nodes ---
        laser_tf,
        driver_with_namespace
    ])
