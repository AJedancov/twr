from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    twr_sim_pkg_path = FindPackageShare('twr_sim')


    # ========================
    # === Launch arguments ===
    # ========================
    sim_launch_arg = DeclareLaunchArgument(
        name='sim',
        default_value='gazebo_sim',
        description='Simulation to launch',
        choices=['gazebo_sim']
    )


    # ============================
    # === Launch configuration ===
    # ============================


    # ============
    # === Sim ====
    # ============
    def launch_sim(context, *args, **kwargs):
        sim_type = context.launch_configurations['sim']

        sim_ld_src = PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                twr_sim_pkg_path, sim_type, 'launch', f'{sim_type}.launch.py'
            ])
        )

        sim_ld = IncludeLaunchDescription(
            launch_description_source=sim_ld_src
        )
        
        return [sim_ld]


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        sim_launch_arg,
    ]

    external_launch_descriptions=[
        OpaqueFunction(function=launch_sim),
    ]

    return LaunchDescription(launch_arguments + external_launch_descriptions)
