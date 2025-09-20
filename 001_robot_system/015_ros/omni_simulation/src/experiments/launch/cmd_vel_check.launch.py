from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = "experiments"

    cmd_vel_check = Node(
        package=pkg_name,
        executable="cmd_vel_check",
        output="screen",
    )

    return LaunchDescription([
        cmd_vel_check,
    ])
