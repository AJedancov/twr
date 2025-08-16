from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    ros_gz_pkg_path = FindPackageShare('ros_gz_sim')
    twr_sim_pkg_path = FindPackageShare('twr_sim')


    # ========================
    # === Launch arguments ===
    # ========================
    gz_sim_world_arg = DeclareLaunchArgument(
        name='gz_sim_world_path',
        default_value=PathJoinSubstitution([
            twr_sim_pkg_path,
            'gazebo_sim',
            'worlds',
            'warehouse.sdf'
        ]),
        description='Path to Gazebo Sim world',
    )


    # ============================
    # === Launch configuration ===
    # ============================
    gz_sim_world = LaunchConfiguration('gz_sim_world_path')


    # ===================
    # === Gazebo Sim ====
    # ===================
    gz_sim_ld_src = PythonLaunchDescriptionSource([
        PathJoinSubstitution([ros_gz_pkg_path, 'launch', 'gz_sim.launch.py'])
    ])

    gz_sim_gui_config = PathJoinSubstitution([
        twr_sim_pkg_path,
        'gazebo_sim',
        'config',
        'gz_gui.config'
    ])
    
    gz_sim_ld_args={'gz_args': ['-r ', gz_sim_world, ' --gui-config ', gz_sim_gui_config]}.items()

    gz_sim_ld = IncludeLaunchDescription(
        launch_description_source=gz_sim_ld_src,
        launch_arguments=gz_sim_ld_args
    )


    # === Gazebo: bridge ===
    gz_bridge_node_config_path = PathJoinSubstitution([
        twr_sim_pkg_path,
        'gazebo_sim', 
        'config',
        'gz_bridge.yaml'
    ])

    gz_bridge_node_params = [{
        'config_file': gz_bridge_node_config_path
    }]

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=gz_bridge_node_params,
 
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
    gz_spawn_entity_node_params = [{
        'name' : 'twr',
        'topic': 'robot_description',
        'z': 0.1
    }]

    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=gz_spawn_entity_node_params
    )


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments=[
        gz_sim_world_arg
    ]

    external_launch_descriptions=[
        gz_sim_ld
    ]

    nodes = [
        gz_spawn_entity_node,
        gz_bridge_node,
    ]

    return LaunchDescription(launch_arguments + external_launch_descriptions + nodes)