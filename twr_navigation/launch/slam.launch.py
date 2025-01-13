import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    slam_toolbox_pkg_path = FindPackageShare('slam_toolbox')
    twr_navigation_pkg_path = FindPackageShare('twr_navigation')

    # === Launch arguments ===
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time',
    )

    # === Launch configuration === 
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')

    # ====================
    # === slam_toolbox ===
    # ====================
    slam_toolbox_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([slam_toolbox_pkg_path, 'launch', 'online_async_launch.py'])
    ])
    
    slam_toolbox_ld_args={'slam_params_file': PathJoinSubstitution([twr_navigation_pkg_path, 'config', 'mapper_params_online_async.yaml']),
                          'use_sim_time': use_sim_time_launch_conf}.items()

    slam_toolbox_ld = IncludeLaunchDescription(
        launch_description_source=slam_toolbox_ld_src,
        launch_arguments=slam_toolbox_ld_args
    )

    ld.add_action(slam_toolbox_ld)
    ld.add_action(use_sim_time_launch_arg)

    return ld