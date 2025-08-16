from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    rsp_pkg_path = FindPackageShare('twr_bringup')
    twr_description_pkg_path = FindPackageShare('twr_description')


    # ========================
    # === Launch arguments ===
    # ========================
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    without_gz_launch_arg = DeclareLaunchArgument(
        name='without_gz',
        default_value='true',
        description='Gazebo is not using',
    )

    rviz2_config_launch_arg = DeclareLaunchArgument(
        name='rviz2_config_path',
        default_value=PathJoinSubstitution([
            twr_description_pkg_path,
            'rviz',
            'nav2_config.rviz'
        ]),
        description='Path to RViz2 configuration file',
    )


    # ============================
    # === Launch configuration ===
    # ============================
    use_sim_time_launch_conf = LaunchConfiguration('use_sim_time')
    rviz2_config_path_launch_conf = LaunchConfiguration('rviz2_config_path')


    # =============
    # === RViz2 ===
    # =============
    rviz2_node_params = [{
        'use_sim_time': use_sim_time_launch_conf
    }]

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_config_path_launch_conf], # -d -> --display-config
        parameters=rviz2_node_params
    )


    # ==============================
    #  === TF Static Publisher =====
    # ==============================
    tf_sb_odom_node_args=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']

    tf_sb_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=tf_sb_odom_node_args,
    )

    tf_sb_map_node_args=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']

    tf_sb_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=tf_sb_map_node_args,
    )


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
    )


    # ==============================
    #  === Joint State Publisher ===
    # ==============================
    # provide JointState messages for robot_state_publisher,
    # also necessary for correct representation of axes in rviz2
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    without_sim_group = GroupAction(
            actions=[rsp_ld, jsp_node],
            condition=IfCondition(LaunchConfiguration('without_gz')),
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        without_gz_launch_arg,
        use_sim_time_launch_arg,
        rviz2_config_launch_arg
    ]

    action_groups=[
        without_sim_group
    ]

    nodes = [
        rviz2_node,
        tf_sb_odom_node,
        tf_sb_map_node
    ]

    return LaunchDescription(launch_arguments + action_groups + nodes)