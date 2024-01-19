from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    #####################################
    # Path settings

    # 1. Underlayer driver launch file path
    # Ref: https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3
    # File Path: velodyne_driver/launch/velodyne_driver_node-VLP16-launch.py
    vlp_driver_path = PathJoinSubstitution([
        FindPackageShare("velodyne_driver"),
        "launch", "velodyne_driver_node-VLP16-launch.py"
    ])

    # 2. Transformation node (raw data -> point cloud)
    # Ref: https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3
    # File Path: velodyne_pointcloud/launch/velodyne_transform_node-VLP16-launch.py
    transformation_path = PathJoinSubstitution([
        FindPackageShare("velodyne_pointcloud"),
        "launch", "velodyne_transform_node-VLP16-launch.py"
    ])

    # 3. Laser scan node (point cloud -> laser scan)
    # Ref: https://github.com/ros-drivers/velodyne/tree/d8cf623a922b1f12995e8c71295924c2905bd9a3
    # File Path: velodyne_laserscan/launch/velodyne_laserscan_node-launch.py
    laserscan_node_path = PathJoinSubstitution([
        FindPackageShare("velodyne_laserscan"),
        "launch", "velodyne_laserscan_node-launch.py"
    ])

    #####################################
    # Add launch description

    vlp_driver = IncludeLaunchDescription(vlp_driver_path)
    transformation = IncludeLaunchDescription(transformation_path)
    laserscan_node = IncludeLaunchDescription(laserscan_node_path)
    
    #####################################
    # Launch description
    return LaunchDescription([
        vlp_driver,       # LiDAR underlayer driver
        transformation,   # Transformation node (raw -> point cloud)
        laserscan_node    # Laser scan node (point cloud -> laser scan)
    ])
