from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = "experiments"

    exp1 = Node(
        package=pkg_name,
        executable="exp1",
        output="screen",
    )

    return LaunchDescription([
        exp1,
    ])
