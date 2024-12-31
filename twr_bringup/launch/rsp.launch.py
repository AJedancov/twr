import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    twr_description_pkg_path = FindPackageShare('twr_description')

    # === Launch arguments ===
    sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Launch RViz',
    )

    # === Launch configuration ===
    sim_time_launch_cfg = LaunchConfiguration('use_sim_time')

    # =============================
    # === Robot State Publisher ===
    # =============================
    twr_xacro_config_file = PathJoinSubstitution([twr_description_pkg_path, 'urdf', 'twr.urdf.xacro'])

    twr_urdf_config_file = Command(['xacro ', twr_xacro_config_file])
    rsp_node_param = {
        'robot_description': twr_urdf_config_file,
        'use_sim_time': sim_time_launch_cfg
    }
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[rsp_node_param]
    )

    ld.add_action(sim_time_launch_arg)
    ld.add_action(rsp_node)

    return ld