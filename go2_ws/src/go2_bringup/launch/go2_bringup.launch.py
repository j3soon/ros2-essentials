from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

ARGUMENTS = [
    DeclareLaunchArgument(
        "platform",
        default_value="isaacsim",
        choices=["isaacsim", "gazebo", "real_robot"],
        description="The platform to launch the robot. Only 'isaacsim' is supported for now.",
    )
]


def generate_launch_description():
    # Set use_sim_time parameter based on the platform
    use_sim_time = SetParameter(
        name="use_sim_time",
        value=PythonExpression(["'false' if '", LaunchConfiguration("platform"), "' == 'real_robot' else 'true'"]),
    )

    # Get Go2 USD file path
    go2_usd_path = PathJoinSubstitution([FindPackageShare("isaacsim"), "assets", "go2_og.usda"])

    # Launch Isaac Sim if the platform is isaacsim
    # Reference:
    # https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_launch.html
    launch_isaacsim = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("isaacsim"),
                "launch",
                "run_isaacsim.launch.py",
            ]
        ),
        launch_arguments={
            "gui": go2_usd_path,
            "dds_type": "fastdds",
            "play_sim_on_start": "true",
            "headless": "standalone",  # "webrtc" or "standalone"
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("platform"), "' == 'isaacsim'"])),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(use_sim_time)
    ld.add_action(launch_isaacsim)

    return ld
