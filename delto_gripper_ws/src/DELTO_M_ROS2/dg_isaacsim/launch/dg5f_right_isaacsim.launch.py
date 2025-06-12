from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true.",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("dg5f_isaacsim"), "urdf", "dg5f_right_isaac.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Load robot controllers configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dg5f_isaacsim"),
            "config",
            "dg5f_right_isaacsim_controller.yaml",
        ]
    )

    # Node for control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Spawner for joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    # Launch joint command convertor node
    joint_command_convertor_node = Node(
        package="dg5f_isaacsim",
        executable="joint_command_convertor.py",
        name="joint_command_convertor",
        output="screen",
    )

    # Launch dg5f right test file
    dg5f_right_test_node = Node(
        package="dg5f_isaacsim",
        executable="dg5f_right_isaacsim_test.py",
        name="dg5f_right_isaacsim_test",
        output="screen",
    )

    # Include RViz2 if GUI is True
    launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Flag to launch rviz.",
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("dg5f_isaacsim"),
            "rviz",
            "dg5f_right_isaacsim.rviz",
        ]
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    nodes = [
        use_sim_time,
        node_robot_state_publisher,
        control_node,
        joint_trajectory_controller_spawner,
        joint_command_convertor_node,
        dg5f_right_test_node,
        launch_rviz,
        rviz2_node,
    ]

    return LaunchDescription(nodes)
