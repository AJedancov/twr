import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    ros_gz_pkg_path = FindPackageShare('ros_gz_sim')
    twr_control_pkg_path = FindPackageShare('twr_control')
    twr_description_pkg_path = FindPackageShare('twr_description')

    # =============================
    # === Robot State Publisher ===
    # =============================
    # TODO: add path to URDF and set as argument for rsp launch file
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

    # TODO: add args for Gazebo Sim launch (gui config, empty.sdf world (with ground))
    gz_sim_ld_args={'gz_args': '-r empty.sdf'}.items()

    gz_sim_ld = IncludeLaunchDescription(
        launch_description_source=gz_sim_ld_src,
        launch_arguments=gz_sim_ld_args
    )

    # === Gazebo: spawn entity ===
    # use /robot_description from robot_state_publisher node
    spawn_entity_node_param = {'name' : 'twr',
                               'topic': 'robot_description'}

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[spawn_entity_node_param]
    )

    # === Gazebo: bridge ===
    bridge_node_param = {'config_file': PathJoinSubstitution([twr_description_pkg_path, 'gz', 'gz_bridge.yaml'])}

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[bridge_node_param],

        # === Instead of a YAML file, we can describe connections as arguments and remappings for them ====
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

    # =============
    # === RViz2 ===
    # =============
    rviz2_node_args = PathJoinSubstitution([
        twr_description_pkg_path, 
        'rviz',
        'config.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_node_args] # -d -> --display-config
    )

    ld.add_action(gz_sim_ld)
    ld.add_action(rsp_ld)
    ld.add_action(spawn_entity_node)
    ld.add_action(bridge_node)
    ld.add_action(rviz2_node)
    
    return ld