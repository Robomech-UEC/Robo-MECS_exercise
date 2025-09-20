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
    robot_name = "orca"

    launch_fname = "setup.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(sim_pkg_name), '/launch/', launch_fname]
        ),
    )

    pd_ctl = Node(
        package=pkg_name,
        executable="pd_ctl",
        namespace=robot_name,
        parameters=[
                {'yaw_kp': 0.1},
                {'yaw_kd': 0.01},
            ],
        output="both",
    )

    quat2yaw = Node(
        package=pkg_name,
        executable="quat2yaw",
        output="screen",
        namespace=robot_name,
    )

    return LaunchDescription([
        sim,
        pd_ctl,
        quat2yaw,
    ])
