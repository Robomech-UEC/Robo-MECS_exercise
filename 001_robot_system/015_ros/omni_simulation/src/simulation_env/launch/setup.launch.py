from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import time


def generate_launch_description():
    # file name and path
    pkg_name = "simulation_env"
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    xacro_file_name = "ORCA_urdf.xacro"

    world_file = os.path.join(
        get_package_share_directory(pkg_name), 'worlds', 'rescon.world'
    )
    world_model_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "worlds"
    ])

    set_model_env = SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=world_model_path
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_pkg, 'launch'), '/gazebo.launch.py']
        ),
        launch_arguments={'world': world_file}.items(),
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(pkg_name), "urdf", xacro_file_name]
            ),
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": robot_description_content,}]
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "rviz",
        "view_robot.rviz"
    ])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        # set_model_env,
        # gazebo,
        # robot_state_publisher,
        # rviz2,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare(pkg_name), '/launch/robot_spawner.launch.py']
            ),
            launch_arguments={'robot_name': 'orca', 'spawn_x': '0.9', 'spawn_y' : '2.25'}.items()
        ),
    ])
