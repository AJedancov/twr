from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    twr_navigation_pkg_path = FindPackageShare('twr_navigation')

    # === Launch arguments ===
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time',
    )

    # === Launch configuration === 
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')

    # ===========
    # === EKF ===
    # ===========

    ekf_node_congig_path = PathJoinSubstitution([
        twr_navigation_pkg_path,
        'robot_localization',
        'config', 
        'ekf.yaml'
    ])

    ekf_node_params = [
        ekf_node_congig_path,
        {'use_sim_time': use_sim_time_launch_conf,}
    ]
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=ekf_node_params,
    )

    ld.add_action(use_sim_time_launch_arg)

    ld.add_action(ekf_node)

    return ld