import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    # === Package Directories ===
    ros_gz_pkg_path = FindPackageShare('ros_gz_sim')
    twr_sim_pkg_path = FindPackageShare('twr_sim')

    # === Launch arguments ===

    # === Launch configuration ===

    # ===================
    # === Gazebo Sim ====
    # ===================
    gz_sim_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([ros_gz_pkg_path, 'launch', 'gz_sim.launch.py'])
    ])

    gz_sim_world = 'empty.sdf'
    gz_sim_gui_config = PathJoinSubstitution([twr_sim_pkg_path, 'config', 'gz_gui.config'])
    
    gz_sim_ld_args={'gz_args': ['-r ', gz_sim_world, ' --gui-config ', gz_sim_gui_config]}.items()

    gz_sim_ld = IncludeLaunchDescription(
        launch_description_source=gz_sim_ld_src,
        launch_arguments=gz_sim_ld_args
    )

    # === Gazebo: bridge ===
    gz_bridge_node_param = {'config_file': PathJoinSubstitution([twr_sim_pkg_path, 'config', 'gz_bridge.yaml'])}

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

    ld.add_action(gz_sim_ld)
    ld.add_action(gz_spawn_entity_node)
    ld.add_action(gz_bridge_node)
    
    return ld