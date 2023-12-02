from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation time",
    )
]


def generate_launch_description():
    # The path of map file.
    map_file = PathJoinSubstitution(
        [FindPackageShare("kobuki_navigation"), "map", "turtlebot3_stage_1.yaml"]
    )

    # Launch robot.
    launch_robot = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("kobuki_launch"), "launch", "kobuki.launch.py"]
        ),
        launch_arguments={
            "is_sim": "True",
            "launch_rviz": "False",
        }.items(),
    )

    # The path of nav2_params file.
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("kobuki_navigation"), "config", "nav2_params.yaml"]
    )

    # Launch nav2.
    launch_nav2 = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"],
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": nav2_params_file,
        }.items(),
    )

    # The path of .rviz file.
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("kobuki_rviz"), "rviz", "navigation.rviz"]
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

    # The path of ekf file.
    ekf_file = PathJoinSubstitution(
        [FindPackageShare("kobuki_navigation"), "config", "ekf.yaml"]
    )

    # Launch robot_localization. (Publish the tf : map -> odom)
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_robot)
    ld.add_action(launch_nav2)
    ld.add_action(launch_rviz)
    ld.add_action(robot_localization_node)

    return ld
