from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    twr_description_pkg_path = FindPackageShare('twr_description')
    
    
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


    # =============================
    # === Robot State Publisher ===
    # =============================
    twr_xacro_config_file = PathJoinSubstitution([twr_description_pkg_path, 'urdf', 'twr.urdf.xacro'])
    twr_urdf_config_file = Command(['xacro ', twr_xacro_config_file])
    
    rsp_node_params = [{
        'robot_description': twr_urdf_config_file,
        'use_sim_time': use_sim_time_launch_conf
    }]
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=rsp_node_params
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        use_sim_time_launch_arg
    ]

    nodes = [
        rsp_node
    ]

    return LaunchDescription(launch_arguments + nodes)