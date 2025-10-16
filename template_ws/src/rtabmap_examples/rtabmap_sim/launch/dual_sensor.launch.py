from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Load the rtabmap settings file
    rtabmap_settings = PathJoinSubstitution([
        FindPackageShare("rtabmap_sim"), 
        "config/dual_sensor.yaml"
    ])

    # Setup the remaps for camera topics
    remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw')
    ]
    
    rtabmap_slam = Node(
        package = 'rtabmap_slam', 
        executable = 'rtabmap',
        parameters = [rtabmap_settings],
        remappings = remappings,
        arguments = ['-d']
    )
    
    rtabmapviz = Node(
        package = 'rtabmap_viz',
        executable = 'rtabmap_viz',
        parameters = [rtabmap_settings],
        remappings = remappings
    )

    return LaunchDescription([

        # Nodes to launch
        rtabmap_slam, rtabmapviz
    ])