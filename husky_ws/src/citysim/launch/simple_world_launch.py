from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set default world path
    world_path = PathJoinSubstitution([
        FindPackageShare("citysim"), 
        "worlds", 
        "simple_city.world"],
    )
    
    # Set gazebo model path
    gz_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            "/usr/share/gazebo-11/models/",
            ":",
            get_package_share_directory("citysim") + "/models",
        ],
    )
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world_path,
        ],
        output="screen",
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    return ld
