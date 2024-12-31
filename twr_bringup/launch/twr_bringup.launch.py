import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    twr_bringup_pkg_path = FindPackageShare('twr_bringup')
    twr_description_pkg_path = FindPackageShare('twr_description')
    twr_control_pkg_path = FindPackageShare('twr_control')
    twr_sim_pkg_path = FindPackageShare('twr_sim')

    # === Launch arguments ===
    use_rviz2_launch_arg = DeclareLaunchArgument(
        name='use_rviz2',
        default_value='True',
        description='Launch RViz2',
    )

    # === Launch configuration ===

    # =============================
    # === Robot State Publisher ===
    # =============================
    rsp_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_bringup_pkg_path, 'launch', 'rsp.launch.py'])
    ])

    rsp_ld_args = {'use_sim_time': 'True'}.items()

    rsp_ld = IncludeLaunchDescription(
        launch_description_source=rsp_ld_src,
        launch_arguments=rsp_ld_args
    )

    # ===============
    # === Control ===
    # ===============
    control_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_control_pkg_path, 'launch', 'control.launch.py'])
    ])

    control_ld = IncludeLaunchDescription(
        launch_description_source=control_ld_src,
    )

    # ============
    # === SLAM ===
    # ============
    slam_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_bringup_pkg_path, 'launch', 'slam.launch.py'])
    ])

    slam_ld = IncludeLaunchDescription(
        launch_description_source=slam_ld_src,
    )

    # =============
    # === RViz2 ===
    # =============
    rviz2_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_bringup_pkg_path, 'launch', 'rviz2.launch.py'])
    ])

    rviz2_ld_args = {'without_gz': 'False'}.items()

    rviz2_ld = IncludeLaunchDescription(
        launch_description_source=rviz2_ld_src,
        launch_arguments=rviz2_ld_args,
        condition=IfCondition(LaunchConfiguration('use_rviz2'))
    )

    # ===================
    # === Simulation ====
    # ===================
    sim_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_sim_pkg_path, 'launch', 'sim.launch.py'])
    ])

    sim_ld = IncludeLaunchDescription(
        launch_description_source=sim_ld_src,
    )


    ld.add_action(use_rviz2_launch_arg)
    
    ld.add_action(rsp_ld)
    ld.add_action(sim_ld)
    ld.add_action(control_ld)
    ld.add_action(rviz2_ld)
    ld.add_action(slam_ld)
    
    return ld