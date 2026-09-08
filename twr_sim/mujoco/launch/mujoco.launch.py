from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare

import xml.etree.ElementTree as ET
import tempfile
from os import close


def generate_launch_description():

    # ===========================
    # === Package directories ===
    # ===========================
    twr_sim_pkg_path = FindPackageShare('twr_sim')
    twr_description_pkg_path = FindPackageShare('twr_description')


    # ========================
    # === Launch arguments ===
    # ========================
    mjc_world_arg = DeclareLaunchArgument(
        name='mjc_world_path',
        default_value=PathJoinSubstitution([
            twr_sim_pkg_path,
            'mujoco',
            'worlds',
            'empty.xml'
        ]),
        description='Path to MuJoCo world',
    )

    mjc_robot_model_arg = DeclareLaunchArgument(
        name='mjc_robot_model_path',
        default_value=PathJoinSubstitution([
            twr_description_pkg_path,
            'mjcf',
            'twr.xml'
        ]),
        description='Path to MuJoCo robot model',
    )


    # ============================
    # === Launch configuration ===
    # ============================


    # ============
    # === Sim ====
    # ============
    def launch_mujoco(context, *args, **kwargs):
        world_path = context.launch_configurations['mjc_world_path']
        robot_model_path = context.launch_configurations['mjc_robot_model_path']

        tree = ET.parse(world_path)
        root = tree.getroot()

        asset = root.find('asset')
        if asset is not None:
            model = ET.Element('model')
            model.set('name', 'robot_model')
            model.set('file', robot_model_path)
            asset.append(model)

        worldbody = root.find('worldbody')
        if worldbody is not None:
            attach = ET.Element('attach')
            attach.set('model', 'robot_model')
            attach.set('prefix', 'attached_')
            worldbody.append(attach)

        temp_fd, temp_mjc_world_path = tempfile.mkstemp(suffix='.xml')
        tree.write(temp_mjc_world_path)
        close(temp_fd)

        mjc_ld = ExecuteProcess(
            cmd=['simulate', temp_mjc_world_path],
            name='mujoco',
            output='screen',
        )

        return [mjc_ld]


    # ==========================
    # === Launch description === 
    # ==========================
    launch_arguments = [
        mjc_world_arg,
        mjc_robot_model_arg,
    ]

    external_launch_descriptions=[
        OpaqueFunction(function=launch_mujoco),
    ]

    return LaunchDescription(launch_arguments + external_launch_descriptions)
