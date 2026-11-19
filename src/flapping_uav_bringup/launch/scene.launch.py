import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def scene_nodes(context):
    share = get_package_share_directory('flapping_uav_bringup')
    with open(LaunchConfiguration('config').perform(context), encoding='utf-8') as stream:
        scene = yaml.safe_load(stream)['scene_visualizer_node']['ros__parameters']
    with open(LaunchConfiguration('uwb_config').perform(context), encoding='utf-8') as stream:
        uwb = yaml.safe_load(stream)['uwb_position_node']['ros__parameters']
    with open(LaunchConfiguration('control_config').perform(context), encoding='utf-8') as stream:
        control = yaml.safe_load(stream)
    if uwb['position_scale'] != 1.0:
        raise RuntimeError('Scene TF requires UWB coordinates already in metres (position_scale=1)')
    scene = dict(scene)
    scene.update(control['frame_contract'])
    scene.update({
        'map_frame_id': uwb['frame_id'],
        'uwb_to_map_yaw_rad': uwb['uwb_to_map_yaw_rad'],
        'uwb_to_map_translation_m': uwb['uwb_to_map_translation_m'],
        'uwb_y_sign': uwb.get('uwb_y_sign', 1.0),
    })
    with open(os.path.join(share, 'config', 'pocket_bench.yaml'), encoding='utf-8') as stream:
        bridge = yaml.safe_load(stream)['mavlink_bridge_node']['ros__parameters']
    bridge = dict(bridge)
    # This optional receiver is DISPLAY ONLY. It never sends heartbeat, RC or 84.
    bridge.update({'tx_enabled': False, 'send_velocity': False})
    return [
        Node(package='flapping_uav_gcs', executable='scene_visualizer_node',
             name='scene_visualizer_node', output='screen', parameters=[scene]),
        Node(package='flapping_uav_gcs', executable='mavlink_bridge_node',
             name='mavlink_bridge_node', output='screen', parameters=[bridge],
             condition=IfCondition(LaunchConfiguration('start_bridge'))),
        Node(package='rviz2', executable='rviz2', name='scene_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'uwb.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'scene.yaml')),
        DeclareLaunchArgument('uwb_config', default_value=os.path.join(share, 'config', 'uwb.yaml')),
        DeclareLaunchArgument('control_config', default_value=os.path.join(share, 'config', 'position_hold.yaml')),
        DeclareLaunchArgument('start_bridge', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=scene_nodes),
    ])
