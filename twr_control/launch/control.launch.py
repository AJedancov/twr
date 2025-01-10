import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    twr_control_pkg_path = FindPackageShare('twr_control')

    # === Launch arguments ===

    # === Launch configuration ===

    # ====================
    # === ros2_control ===
    # ====================
    twr_controllers = PathJoinSubstitution([twr_control_pkg_path, 'config', 'twr_diff_drive_controller.yaml'])

    # controller_manager commented -> gz_ros2_control runs the controller_manager, no need for ros2_control_node

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[twr_controllers],
    #     output="both",
    #     # remappings=[
    #     #     ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
    #     # ],
    # )
    # ld.add_action(control_node)

    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            twr_controllers,
        ],
    )

    ld.add_action(joint_state_broadcaster_spawner_node)
    ld.add_action(diff_drive_base_controller_spawner_node)
    
    return ld