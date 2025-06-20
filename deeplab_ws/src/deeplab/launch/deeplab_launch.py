from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "input_image_topic", 
        default_value="/realsense/image_raw",
        description="The input image topic for inference."
    ),
    DeclareLaunchArgument(
        "output_image_topic", 
        default_value="/deeplab/result",
        description="The output image topic for node."
    ),
]


def generate_launch_description():
    # Spawn Node
    deeplab_node = Node(
        package="deeplab",
        executable="inference.py",
        name="deeplab_node",
        output="screen",
        remappings=[
            ("/input", LaunchConfiguration("input_image_topic")),
            ("/output", LaunchConfiguration("output_image_topic")),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(deeplab_node)

    return ld
