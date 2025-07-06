from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument("world_path", default_value="",
                          description="The world path, by default is empty.world"),
]


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=[
                                                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
                                                "/usr/share/gazebo-11/models/",
                                                ":",
                                                str(Path(get_package_share_directory("husky_description")).parent.resolve()),
                                                ":",
                                                str(Path(get_package_share_directory("realsense2_description")).parent.resolve())])

    # Launch args
    world_path = LaunchConfiguration("world_path")
    prefix = LaunchConfiguration("prefix")
    
    # Launch Husky's description. ( Use the launch file in "husky_description" )
    # It retrieves Husky's URDF via xacro and publishes the robot state.
    launch_husky_description = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("husky_description"), "launch", "description_launch.py"]
        )
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["husky_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=["gzserver",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so",
             world_path],
        output="screen",
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        # condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_husky",
        arguments=["-entity",
                   "husky",
                   "-topic",
                   "robot_description"],
        output="screen",
    )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), "launch", "control.launch.py"])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), "launch", "teleop_base.launch.py"])))

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(launch_husky_description)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)

    return ld
