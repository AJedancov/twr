import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    ros_gz_pkg_path = FindPackageShare('ros_gz_sim')
    twr_control_pkg_path = FindPackageShare('twr_control')
    twr_description_pkg_path = FindPackageShare('twr_description')
    twr_sim_pkg_path = FindPackageShare('twr_sim')

    # === Launch arguments ===
    use_rviz2_launch_arg = DeclareLaunchArgument(
        name='use_rviz2',
        default_value='True',
        description='Launch RViz',
    )

    # =============================
    # === Robot State Publisher ===
    # =============================
    rsp_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_control_pkg_path, 'launch', 'rsp.launch.py'])
    ])

    rsp_ld_args = {'use_sim_time': 'True'}.items()

    rsp_ld = IncludeLaunchDescription(
        launch_description_source=rsp_ld_src,
        launch_arguments=rsp_ld_args
    )

    # ===================
    # === Gazebo Sim ====
    # ===================
    gz_sim_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([ros_gz_pkg_path, 'launch', 'gz_sim.launch.py'])
    ])

    gz_sim_world = 'empty.sdf'
    gz_sim_gui_config = PathJoinSubstitution([twr_description_pkg_path, 'gz', 'gz_gui.config'])
    
    gz_sim_ld_args={'gz_args': ['-r ', gz_sim_world, ' --gui-config ', gz_sim_gui_config]}.items()

    gz_sim_ld = IncludeLaunchDescription(
        launch_description_source=gz_sim_ld_src,
        launch_arguments=gz_sim_ld_args
    )

    # === Gazebo: bridge ===
    gz_bridge_node_param = {'config_file': PathJoinSubstitution([twr_description_pkg_path, 'gz', 'gz_bridge.yaml'])}

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[gz_bridge_node_param],

        #  Instead of a YAML file, we can describe connections as arguments and remappings for them 
        # 
        # arguments=[
        #     # Clock (GZ -> ROS2)
        #     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        #     # Joint states (GZ -> ROS2)
        #     '/world/empty/model/twr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        #     # TF (GZ -> ROS2)
        #     '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        #     '/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        #     # Velocity and odometry (Gazebo -> ROS2)
        #     '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        #     '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        # ],
        # remappings=[
        #     ('/world/empty/model/twr/joint_state', 'joint_states'),
        #     ('/world/empty/dynamic_pose/info', 'tf'),
        #     ('/world/empty/pose/info', 'tf_static')
        # ],
    )

    # === Gazebo: spawn entity ===
    # use /robot_description from robot_state_publisher node
    gz_spawn_entity_node_param = {'name' : 'twr',
                                  'topic': 'robot_description'}

    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[gz_spawn_entity_node_param]
    )

    # === ROS2 Controllers ===
    twr_controllers = PathJoinSubstitution([twr_description_pkg_path, 'controllers', 'twr_diff_drive.yaml'])

    # gz_ros2_control runs the controller_manager, no need for ros2_control_node

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[twr_controllers],
    #     output="both",
    #     # remappings=[
    #     #     ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
    #     # ],
    # )

    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            twr_controllers,
            ],
    )

    # =============
    # === RViz2 ===
    # =============

    rviz2_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_sim_pkg_path, 'launch', 'rviz2.launch.py'])
    ])

    rviz2_ld_args = {'without_gz': 'False'}.items()

    rviz2_ld = IncludeLaunchDescription(
        launch_description_source=rviz2_ld_src,
        launch_arguments=rviz2_ld_args,
        condition=IfCondition(LaunchConfiguration('use_rviz2'))
    )


    # ============
    # === SLAM ===
    # ============
    slam_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_sim_pkg_path, 'launch', 'slam.launch.py'])
    ])

    slam_ld = IncludeLaunchDescription(
        launch_description_source=slam_ld_src,
    )


    ld.add_action(use_rviz2_launch_arg)
    ld.add_action(gz_sim_ld)
    ld.add_action(rsp_ld)
    ld.add_action(gz_spawn_entity_node)
    ld.add_action(gz_bridge_node)
    # ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner_node)
    ld.add_action(diff_drive_base_controller_spawner_node)
    ld.add_action(rviz2_ld)
    ld.add_action(slam_ld)
    
    return ld