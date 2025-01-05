import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    slam_toolbox_pkg_path = FindPackageShare('slam_toolbox')
    twr_description_pkg_path = FindPackageShare('twr_description')

    # === Launch arguments ===

    # === Launch configuration === 

    # ====================
    # === slam_toolbox ===
    # ====================
    slam_toolbox_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([slam_toolbox_pkg_path, 'launch', 'online_async_launch.py'])
    ])
    
    slam_toolbox_ld_args={'slam_params_file': PathJoinSubstitution([twr_description_pkg_path, 'slam_toolbox', 'mapper_params_online_async.yaml']),
                          'use_sim_time': 'True'}.items()

    slam_toolbox_ld = IncludeLaunchDescription(
        launch_description_source=slam_toolbox_ld_src,
        launch_arguments=slam_toolbox_ld_args
    )

    ld.add_action(slam_toolbox_ld)

    return ld