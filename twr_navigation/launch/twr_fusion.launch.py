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
        choices=["true", "false"]
    )


    # ============================
    # === Launch configuration ===
    # ============================
    use_sim_time_launch_config = LaunchConfiguration('use_sim_time')


    # ==========================
    # === robot_localization ===
    # ==========================
    ekf_map_to_odom_node_config_path = PathJoinSubstitution([
        twr_navigation_pkg_path,
        'robot_localization',
        'config', 
        'ekf_map_to_odom.yaml'
    ])

    ekf_odom_to_base_node_config_path = PathJoinSubstitution([
        twr_navigation_pkg_path,
        'robot_localization',
        'config', 
        'ekf_odom_to_base.yaml'
    ])

    ekf_map_to_odom_node_params = [
        ekf_map_to_odom_node_config_path,
        {'use_sim_time': use_sim_time_launch_config,}
    ]

    ekf_odom_to_base_node_params = [
        ekf_odom_to_base_node_config_path,
        {'use_sim_time': use_sim_time_launch_config,}
    ]

    ekf_node_remaps = [('odometry/filtered', 'odom')]

    ekf_map_to_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_to_odom',
        parameters=ekf_map_to_odom_node_params,
        remappings=ekf_node_remaps
    )

    ekf_odom_to_base_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom_to_base',
        parameters=ekf_odom_to_base_node_params,
        remappings=ekf_node_remaps
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        use_sim_time_launch_arg
    ]

    nodes = [
        ekf_map_to_odom_node,
        ekf_odom_to_base_node
    ]

    return LaunchDescription(launch_arguments + nodes)