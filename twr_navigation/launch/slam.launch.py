from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    twr_navigation_pkg_path = FindPackageShare('twr_navigation')


    # ========================
    # === Launch arguments ===
    # ========================
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    slam_toolbox_node_params_path_launch_arg = DeclareLaunchArgument(
        name='slam_toolbox_node_params_path',
        default_value=PathJoinSubstitution([
            twr_navigation_pkg_path, 
            'slam_toolbox',
            'config',
            'online_async_localization_params.yaml'
        ]),
        description='Path to slam_toolbox parameters',
    )

    slam_toolbox_path_to_map_launch_arg = DeclareLaunchArgument(
        name='slam_toolbox_path_to_map',
        default_value=PathJoinSubstitution([
            twr_navigation_pkg_path, 
            'map',
            'warehouse',
            'warehouse_serial_map'
        ]),
        description='Path to map for slam_toolbox. Specify without file extension, only file name',
    )


    # ============================
    # === Launch configuration ===
    # ============================
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')
    slam_toolbox_node_params_path_launch_conf = LaunchConfiguration('slam_toolbox_node_params_path')
    slam_toolbox_path_to_map_launch_conf = LaunchConfiguration('slam_toolbox_path_to_map')


    # ====================
    # === slam_toolbox ===
    # ====================
    slam_toolbox_async_node_params = [
        slam_toolbox_node_params_path_launch_conf,
        {
            'use_sim_time': use_sim_time_launch_conf,
            'map_file_name': slam_toolbox_path_to_map_launch_conf
        }
    ]

    # This node is a lifecycle node. 
    # Its configuration is performed via nav2_lifecycle_manager
    slam_toolbox_async_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=slam_toolbox_async_node_params,
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        use_sim_time_launch_arg,
        slam_toolbox_node_params_path_launch_arg,
        slam_toolbox_path_to_map_launch_arg,
    ]

    nodes = [
        slam_toolbox_async_node
    ]

    return LaunchDescription(launch_arguments + nodes)