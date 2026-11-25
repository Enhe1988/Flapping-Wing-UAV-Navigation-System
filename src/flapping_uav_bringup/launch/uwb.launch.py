import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def hardware_nodes(context):
    port = LaunchConfiguration('port_name').perform(context)
    baud_text = LaunchConfiguration('baud_rate').perform(context)
    try:
        baud = int(baud_text)
    except ValueError:
        baud = 0
    if not port or baud <= 0:
        raise RuntimeError('Confirm hardware first: pass port_name:=<device> and '
                           'baud_rate:=<actual baud>. No default device or baud is assumed.')
    share = get_package_share_directory('flapping_uav_bringup')
    config_path = LaunchConfiguration('config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    try:
        parameters = dict(config['uwb_position_node']['ros__parameters'])
    except (TypeError, KeyError):
        raise RuntimeError('Config must contain uwb_position_node.ros__parameters')
    confirmed = LaunchConfiguration('coordinates_confirmed').perform(context).lower()
    if confirmed not in ('true', 'false'):
        raise RuntimeError('coordinates_confirmed must be true or false')
    # Foxy may prioritize a node-specific YAML selector over a later wildcard
    # parameter dictionary. Merge first and hand Node a single dictionary.
    parameters['coordinates_confirmed'] = confirmed == 'true'
    return [
        Node(package='flapping_uav_gcs', executable='linktrack_host_node',
             name='linktrack_parser', output='screen',
             parameters=[{'port_name': port, 'baud_rate': baud}]),
        Node(package='flapping_uav_gcs', executable='uwb_position_node',
             name='uwb_position_node', output='screen',
             parameters=[parameters]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'scene.launch.py')),
            launch_arguments={
                'config': LaunchConfiguration('scene_config'),
                'uwb_config': config_path,
                'control_config': LaunchConfiguration('control_config'),
                'scene_position_topic': LaunchConfiguration('scene_position_topic'),
                'use_rviz': 'false', 'start_bridge': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_scene')))]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'uwb.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value=''),
        DeclareLaunchArgument('baud_rate', default_value='0'),
        DeclareLaunchArgument('coordinates_confirmed', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_scene', default_value='true'),
        DeclareLaunchArgument('scene_position_topic', default_value='/uwb/position'),
        DeclareLaunchArgument('scene_config', default_value=os.path.join(share, 'config', 'scene.yaml')),
        DeclareLaunchArgument('control_config', default_value=os.path.join(share, 'config', 'position_hold.yaml')),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'uwb.yaml')),
        OpaqueFunction(function=hardware_nodes),
    ])
