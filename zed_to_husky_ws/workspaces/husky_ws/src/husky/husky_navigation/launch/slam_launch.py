from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Enable use_sim_time",
    )
]


def generate_launch_description():
    # Launch gazebo. ( Use the launch file in "husky_gazebo" )
    launch_husky_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("husky_gazebo"), "launch", "husky_playpen.launch.py"]
        )
    )

    # Launch SLAM toolbox. ( Use the launch file in "slam_toolbox" )
    launch_slam_toolbox = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("slam_toolbox").find("slam_toolbox"), "launch", "online_async_launch.py"]
        )
    )
    
    # The path of nav2_params file.
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("husky_navigation"), "config", "nav2_params.yaml"]
    )

    # Launch nav2_bringup. ( Use the launch file in "nav2_bringup" )
    launch_nav2_bringup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("nav2_bringup").find("nav2_bringup"), "launch", "navigation_launch.py"]
        ),
        launch_arguments={"params_file": nav2_params_file}.items()
    )

    # Launch rviz2. ( Use the launch file in "husky_viz" )
    launch_husky_viz = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("husky_viz"), "launch", "view_robot_launch.py"]
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_husky_gazebo)
    ld.add_action(launch_slam_toolbox)
    ld.add_action(launch_nav2_bringup)
    ld.add_action(launch_husky_viz)

    return ld