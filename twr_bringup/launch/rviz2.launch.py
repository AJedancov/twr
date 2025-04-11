import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    rsp_pkg_path = FindPackageShare('twr_bringup')
    twr_description_pkg_path = FindPackageShare('twr_description')

    # === Launch arguments ===
    without_gz_launch_arg = DeclareLaunchArgument(
        name='without_gz',
        default_value='True',
        description='Gazebo is not using',
    )

    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time',
    )

    # === Launch configuration === 
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')

    # =============
    # === RViz2 ===
    # =============

    rviz2_node_args = PathJoinSubstitution([
        twr_description_pkg_path, 
        'rviz',
        'nav2_config.rviz'
    ])

    rviz2_node_params = {
        'use_sim_time': use_sim_time_launch_conf
    }

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_node_args], # -d -> --display-config
        parameters=[rviz2_node_params]
    )
    
    ld.add_action(without_gz_launch_arg)
    ld.add_action(use_sim_time_launch_arg)
    ld.add_action(rviz2_node)


    # =============================
    # === Robot State Publisher ===
    # =============================
    
    rsp_ld_source = PythonLaunchDescriptionSource([
        PathJoinSubstitution([rsp_pkg_path, 'launch', 'rsp.launch.py'])
    ])

    rsp_ld_args = {'use_sim_time': use_sim_time_launch_conf}.items()

    rsp_ld = IncludeLaunchDescription(
        launch_description_source=rsp_ld_source,
        launch_arguments=rsp_ld_args,
        condition=IfCondition(LaunchConfiguration('without_gz'))
    )

    # ==============================
    #  === Joint State Publisher ===
    # ==============================

    # provide JointState messages for robot_state_publisher,
    # also necessary for correct representation of axes in rviz2
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('without_gz'))
    )


    ld.add_action(rsp_ld)
    ld.add_action(jsp_node)

    
    return ld