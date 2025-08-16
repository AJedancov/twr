from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    twr_bringup_pkg_path = FindPackageShare('twr_bringup')
    twr_control_pkg_path = FindPackageShare('twr_control')
    twr_navigation_pkg_path = FindPackageShare('twr_navigation')
    twr_sim_pkg_path = FindPackageShare('twr_sim')


    # ========================
    # === Launch arguments ===
    # ========================
    use_rviz2_launch_arg = DeclareLaunchArgument(
        name='use_rviz2',
        default_value='true',
        description='Launch RViz2',
    )

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
    rsp_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_bringup_pkg_path, 'launch', 'rsp.launch.py'])
    ])

    rsp_ld_args = {'use_sim_time': use_sim_time_launch_conf}.items()

    rsp_ld = IncludeLaunchDescription(
        launch_description_source=rsp_ld_src,
        launch_arguments=rsp_ld_args,
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
        PathJoinSubstitution([twr_navigation_pkg_path, 'launch', 'slam.launch.py'])
    ])

    slam_ld_args = {'use_sim_time': use_sim_time_launch_conf}.items()

    slam_ld = IncludeLaunchDescription(
        launch_description_source=slam_ld_src,
        launch_arguments=slam_ld_args,
    )


    # =====================
    # === Sensor Fusion ===
    # =====================
    fusion_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_navigation_pkg_path, 'launch', 'fusion.launch.py'])
    ])

    fusion_ld_args = {'use_sim_time': use_sim_time_launch_conf}.items()

    fusion_ld = IncludeLaunchDescription(
        launch_description_source=fusion_ld_src,
        launch_arguments=fusion_ld_args,
    )


    # ==================
    # === Navigation ===
    # ==================
    navigation_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_navigation_pkg_path, 'launch', 'navigation.launch.py'])
    ])

    navigation_ld_args = {'use_sim_time': use_sim_time_launch_conf}.items()

    navigation_ld = IncludeLaunchDescription(
        launch_description_source=navigation_ld_src,
        launch_arguments=navigation_ld_args,
    )


    # =============
    # === RViz2 ===
    # =============
    rviz2_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_bringup_pkg_path, 'launch', 'rviz2.launch.py'])
    ])

    rviz2_ld_args = {
        'without_gz': 'False',
        'use_sim_time': use_sim_time_launch_conf}.items()

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


    # ==============================
    # === Nav2 Lifecycle Manager ===
    # ==============================
    nav2_lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'slam_toolbox'
    ]

    nav2_lifecycle_manager_params = [{
        'autostart': True,
        'node_names': nav2_lifecycle_nodes,
    }]

    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=nav2_lifecycle_manager_params,
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        use_rviz2_launch_arg,
        use_sim_time_launch_arg
    ]

    external_launch_descriptions=[
        rsp_ld,
        sim_ld,
        control_ld,
        navigation_ld,
        rviz2_ld,
        slam_ld,
        fusion_ld
    ]

    nodes = [
        nav2_lifecycle_manager_node
    ]

    return LaunchDescription(launch_arguments + external_launch_descriptions + nodes)