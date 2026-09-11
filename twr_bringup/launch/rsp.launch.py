from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
        choices=["true", "false"]
    )

    sim_launch_arg = DeclareLaunchArgument(
        name='sim',
        default_value='gazebo_sim',
        description='Simulation to launch',
        choices=['gazebo_sim', 'mujoco']
    )


    # ============================
    # === Launch configuration ===
    # ============================
    use_sim_time_launch_config = LaunchConfiguration('use_sim_time')


    # =============================
    # === Robot State Publisher ===
    # =============================
    def launch_rsp_node(context, *args, **kwargs):
        sim_type = context.launch_configurations['sim']

        twr_xacro_config_file = PathJoinSubstitution([twr_description_pkg_path, 'urdf', 'twr.urdf.xacro'])
        twr_urdf_config_file = Command([
            'xacro ',
            twr_xacro_config_file, 
            f' sim:={sim_type}'
        ])

        rsp_node_params = [{
            'robot_description': twr_urdf_config_file,
            'use_sim_time': use_sim_time_launch_config
        }]
        
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=rsp_node_params
        )
        return [rsp_node]


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        use_sim_time_launch_arg,
        sim_launch_arg,
    ]

    nodes = [
        OpaqueFunction(function=launch_rsp_node)
    ]

    return LaunchDescription(launch_arguments + nodes)