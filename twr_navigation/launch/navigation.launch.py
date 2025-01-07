import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # === Package directories ===
    twr_navigation_pkg_path = FindPackageShare('twr_navigation')
    nav2_bringup_pkg_path = FindPackageShare('nav2_bringup')
    

    # === Launch arguments ===
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time',
    )

    slam_launch_arg = DeclareLaunchArgument(
        name='slam', 
        default_value='True', 
        description='Use SLAM'
    )

    params_file_launch_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=PathJoinSubstitution([twr_navigation_pkg_path, 'config', 'nav2.yaml']),
        description='Path to Nav2 parameters',
    )

    # === Launch configuration === 
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')
    slam_launch_conf = LaunchConfiguration('slam')
    params_file_launch_conf = LaunchConfiguration('params_file')

    # ============
    # === Nav2 ===
    # ============
    nav2_bringup_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([nav2_bringup_pkg_path, 'launch', 'bringup_launch.py'])
    ])
    
    nav2_bringup_ld_args={
            'use_sim_time': use_sim_time_launch_conf,
            'slam': slam_launch_conf,
            'params_file': params_file_launch_conf,
        }.items()

    nav2_bringup_ld = IncludeLaunchDescription(
        launch_description_source=nav2_bringup_src,
        launch_arguments=nav2_bringup_ld_args
    )

    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(slam_launch_arg)
    ld.add_action(params_file_launch_arg)

    ld.add_action(nav2_bringup_ld)

    return ld