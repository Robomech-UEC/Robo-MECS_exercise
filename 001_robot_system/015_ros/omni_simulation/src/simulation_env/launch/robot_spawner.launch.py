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
import os
import xacro

def launch_setup(context, *args, **keywargs):
    robot_name = str( LaunchConfiguration('robot_name').perform(context) )
    spawn_x = str( LaunchConfiguration('spawn_x').perform(context) )
    spawn_y = str( LaunchConfiguration('spawn_y').perform(context) )

    # file name and path
    sim_pkg_name = "simulation_env"
    
    xacro_file_path = os.path.join(get_package_share_directory(sim_pkg_name), "urdf", "ORCA_urdf.xacro")
    xacro_namespace_attached = xacro.process_file(xacro_file_path, mappings={'robot_name' : robot_name})
    robot_description_content = xacro_namespace_attached.toxml()

    # spawnner
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f"{robot_name}_robot_state_publisher",
        parameters=[{"robot_description": robot_description_content,}],
        remappings=[('/robot_description', f'/{robot_name}_robot_description'),
                    ('/joint_states', f'/{robot_name}/joint_states'),],
        # namespace=robot_name,
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', f'/{robot_name}_robot_description',
                   '-entity', f'{robot_name}',
                   '-x', f'{spawn_x}', '-y', f'{spawn_y}', '-z', '0.1',],
        remappings=[("/robot_state_publisher", f"{robot_name}_robot_state_publisher"),],
        # namespace=robot_name,
        output='screen',
    )

    # controllers
    controller_remappings = [("/controller_manager/list_controllers", f"/{robot_name}/controller_manager/list_controllers"),
                             ("/controller_manager/load_controller", f"/{robot_name}/controller_manager/load_controller"),
                             ("/controller_manager/configure_controller", f"/{robot_name}/controller_manager/configure_controller"),
                             ("/controller_manager/switch_controller", f"/{robot_name}/controller_manager/switch_controller"),]

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_controller", "--controller-manager", "/controller_manager"],
        remappings=controller_remappings,
        # namespace=robot_name,
        output="screen",
    )

    suspension_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["suspension_controller", "--controller-manager", "/controller_manager"],
        remappings=controller_remappings,
        # namespace=robot_name,
        output="screen",
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_controller", "--controller-manager", "/controller_manager"],
        remappings=controller_remappings,
        # namespace=robot_name,
        output="screen",
    )

    # robot control
    robot_executor = Node(
        package=sim_pkg_name,
        executable="robot_executor",
        namespace=robot_name,
        output="screen",
    )

    return [# gazebo rviz
            robot_state_publisher,
            spawn_entity,
            # controllers
            wheel_controller_spawner,
            suspension_controller_spawner,
            servo_controller_spawner,
            # robot control
            robot_executor,
            ]

def generate_launch_description():
    # external input
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='orca')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.45')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.45')

    return LaunchDescription([
        robot_name_arg, spawn_x_arg, spawn_y_arg,
        OpaqueFunction(function = launch_setup),
    ])
