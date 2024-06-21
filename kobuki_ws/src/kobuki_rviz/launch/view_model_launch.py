from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation time",
    )
]


def generate_launch_description():
    # Launch joint state publisher.
    node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # The path of .rviz file.
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("kobuki_rviz"), "rviz", "view_model.rviz"]
    )

    # Launch rviz2.
    launch_rviz = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_rviz"),
                "launch",
                "open_rviz_launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz_config_path": rviz_config_path,
        }.items(),
    )

    # Launch kobuki's description.
    launch_kobuki_description = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("kobuki_description"),
                "launch",
                "robot_description.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_joint_state_publisher_gui)
    ld.add_action(launch_rviz)
    ld.add_action(launch_kobuki_description)

    return ld
