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


    # ============================
    # === Launch configuration ===
    # ============================
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')


    # ==========================
    # === robot_localization ===
    # ==========================
    ekf_node_config_path = PathJoinSubstitution([
        twr_navigation_pkg_path,
        'robot_localization',
        'config', 
        'ekf.yaml'
    ])

    ekf_node_params = [
        ekf_node_config_path,
        {'use_sim_time': use_sim_time_launch_conf,}
    ]

    ekf_node_remaps = [('odometry/filtered', 'odom')]
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=ekf_node_params,
        remappings=ekf_node_remaps
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        use_sim_time_launch_arg
    ]

    nodes = [
        ekf_node
    ]

    return LaunchDescription(launch_arguments + nodes)