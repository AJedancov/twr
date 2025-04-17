import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # === Package directories ===
    twr_navigation_pkg_path = FindPackageShare('twr_navigation')

    # === Launch arguments ===
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time',
    )

    # === Launch configuration === 
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')

    # ============
    # === Nav2 ===
    # ============
    common_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    cmd_vel_remappings = [('cmd_vel', 'diff_drive_controller/cmd_vel')]
    
    # === Behaviour Tree Server ===
    nav2_bt_params = [
        PathJoinSubstitution([twr_navigation_pkg_path, 'config', 'nav2_bt.yaml']),
        {'use_sim_time': use_sim_time_launch_conf,}
    ]

    nav2_behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=nav2_bt_params,
        remappings=common_remappings + cmd_vel_remappings,
    )

    # === Behaviour Tree Navigator ===
    nav2_bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=nav2_bt_params,
        remappings=common_remappings,
    )
    
    # === Planner Server ===
    nav2_planner_server_params = [
        PathJoinSubstitution([twr_navigation_pkg_path, 'config', 'nav2_planner.yaml']),
        {'use_sim_time': use_sim_time_launch_conf,}
    ]

    nav2_planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=nav2_planner_server_params,
        remappings=common_remappings,
    )

    # === Controller Server ===
    nav2_controller_server_params = [
        PathJoinSubstitution([twr_navigation_pkg_path, 'config', 'nav2_controller_mppi.yaml']),
        {'use_sim_time': use_sim_time_launch_conf,}
    ]
    
    nav2_controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=nav2_controller_server_params,
        remappings=common_remappings + cmd_vel_remappings,
    )

    # === Lifecycle Manager === 
    nav2_lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    nav2_lifecycle_manager_params = [
        {'autostart': True,
        'node_names': nav2_lifecycle_nodes,}

    ]

    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=nav2_lifecycle_manager_params,
    )

    ld.add_action(nav2_behavior_server_node)
    ld.add_action(nav2_bt_navigator_node)
    ld.add_action(nav2_planner_server_node)
    ld.add_action(nav2_controller_server_node)
    ld.add_action(nav2_lifecycle_manager_node)
    
    return ld