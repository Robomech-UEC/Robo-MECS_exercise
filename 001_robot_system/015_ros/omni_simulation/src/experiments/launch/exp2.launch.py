from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

def generate_launch_description():
    pkg_name = "experiments"
    sim_pkg_name = "simulation_env"

    launch_fname = "setup.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(sim_pkg_name), '/launch/', launch_fname]
        ),
    )

    exp1 = Node(
        package=pkg_name,
        executable="exp2",
        output="screen",
    )

    return LaunchDescription([
        sim,
        exp1,
    ])
