import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

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

    nav2_planner_server_prefix_launch_arg = DeclareLaunchArgument(
        name='nav2_planner_server_prefix',
        default_value='',
        description='Prefix for Nav2 planner_server node. Declare this argument as a string in single or double quotes: \'prefix_content\' or \"prefix_content\"',
    )
    
    nav2_planner_server_params_path_launch_arg = DeclareLaunchArgument(
        name='nav2_planner_server_params_path',
        default_value=PathJoinSubstitution([
            twr_navigation_pkg_path, 
            'nav2_planners',
            'nav2_navfn_planner',
            'config',
            'nav2_navfn_planner_params.yaml'
        ]),
        description='Path to Nav2 planner server parameters',
    )

    nav2_global_costmap_params_path_launch_arg = DeclareLaunchArgument(
        name='nav2_global_costmap_params_path',
        default_value=PathJoinSubstitution([
            twr_navigation_pkg_path,
            'nav2_planners',
            'nav2_global_costmap',
            'nav2_global_costmap.yaml'
        ]),
        description='Path to Nav2 global costmap parameters',
    )

    # === Launch configuration === 
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')
    nav2_planner_server_prefix_launch_conf = LaunchConfiguration('nav2_planner_server_prefix')
    nav2_planner_server_params_path_launch_conf = LaunchConfiguration('nav2_planner_server_params_path')
    nav2_global_costmap_params_path_launch_conf = LaunchConfiguration('nav2_global_costmap_params_path')


    # ============
    # === Nav2 ===
    # ============

    common_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    cmd_vel_remappings = [('cmd_vel', 'diff_drive_controller/cmd_vel')]
        
    # === Planner Server ===
    nav2_planner_server_params = [
        nav2_planner_server_params_path_launch_conf,
        nav2_global_costmap_params_path_launch_conf,
        {'use_sim_time': use_sim_time_launch_conf,}
    ]

    nav2_planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        prefix=nav2_planner_server_prefix_launch_conf,
        parameters=nav2_planner_server_params,
        remappings=common_remappings,
    )


    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(nav2_planner_server_prefix_launch_arg)
    ld.add_action(nav2_planner_server_params_path_launch_arg)
    ld.add_action(nav2_global_costmap_params_path_launch_arg)

    ld.add_action(nav2_planner_server_node)
    
    return ld