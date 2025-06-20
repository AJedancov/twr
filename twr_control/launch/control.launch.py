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
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time',
    )
    
    nav2_controller_server_params_path_launch_arg = DeclareLaunchArgument(
        name='nav2_controller_server_params_path',
        default_value=PathJoinSubstitution([
            twr_control_pkg_path,
            'nav2_controllers',
            'nav2_mppi_controller',
            'config',
            'nav2_mppi_controller_params.yaml'
        ]),
        description='Path to Nav2 controller server parameters',
    )

    nav2_local_costmap_params_path_launch_arg = DeclareLaunchArgument(
        name='nav2_local_costmap_params_path',
        default_value=PathJoinSubstitution([
            twr_control_pkg_path,
            'nav2_controllers',
            'nav2_local_costmap',
            'nav2_local_costmap.yaml'
        ]),
        description='Path to Nav2 local costmap parameters',
    )

    # === Launch configuration ===
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')
    nav2_controller_server_params_path_launch_conf = LaunchConfiguration('nav2_controller_server_params_path')
    nav2_local_costmap_params_path_launch_conf = LaunchConfiguration('nav2_local_costmap_params_path')


    # ====================
    # === ros2_control ===
    # ====================
    twr_diff_drive_controller_params = PathJoinSubstitution([
        twr_control_pkg_path,
        'ros2_controllers',
        'diff_drive_controller',
        'config', 
        'twr_diff_drive_controller.yaml'
    ])

    # === commented === 
    # gz_ros2_control runs the controller_manager, no need for ros2_control_node

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[twr_diff_drive_controller_params],
    #     output="both"
    # )
    # ld.add_action(control_node)
    # =================

    spawner_node_args = [
        'joint_state_broadcaster',
        'diff_drive_controller',
            '--param-file',
            twr_diff_drive_controller_params,
    ]

    spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_node_args,
    )

    # ====================
    # === Nav2 Control ===
    # ====================

    common_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    cmd_vel_remappings = [('cmd_vel', 'diff_drive_controller/cmd_vel')]
    
    # === Controller Server ===
    nav2_controller_server_params = [
        nav2_controller_server_params_path_launch_conf,
        nav2_local_costmap_params_path_launch_conf,
        {'use_sim_time': use_sim_time_launch_conf,}
    ]
    
    nav2_controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=nav2_controller_server_params,
        remappings=common_remappings + cmd_vel_remappings,
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(nav2_controller_server_params_path_launch_arg)
    ld.add_action(nav2_local_costmap_params_path_launch_arg)

    ld.add_action(spawner_node)
    ld.add_action(nav2_controller_server_node)
    
    return ld