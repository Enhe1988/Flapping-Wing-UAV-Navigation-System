import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    def flag(key):
        text = value(key).lower()
        if text not in ('true', 'false'):
            raise RuntimeError(key + ' must be true or false')
        return text == 'true'
    with open(value('config'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    use_rc = flag('use_rc')
    parameters = {'use_joy': use_rc and flag('use_joy')}
    for prefix, key in [('joy', 'joy_node'), ('manual', 'manual_control_node'),
                        ('bridge', 'mavlink_bridge_node')]:
        parameters.update({prefix + '.' + k: v for k, v in config[key]['ros__parameters'].items()})
    # Use the same control profile for FC attitude tolerance as position_hold.
    with open(value('control_config'), encoding='utf-8') as stream:
        control = yaml.safe_load(stream)
    parameters.update({'bridge.' + k: v for k, v in
                       control.get('mavlink_bridge_node', {}).get('ros__parameters', {}).items()})
    protocol=value('velocity_protocol')
    if protocol!='config':
        if protocol not in ('legacy','compact'):
            raise RuntimeError('velocity_protocol must be config, legacy or compact')
        parameters['bridge.velocity_protocol']=protocol
        parameters['bridge.attitude_message_id']=254 if protocol=='compact' else 31
    mode=value('horizontal_control_mode')
    if mode=='config':
        mode='velocity' if protocol=='legacy' else control['position_controller_node']['ros__parameters'].get('horizontal_control_mode','velocity')
    if mode not in ('velocity','attitude'):
        raise RuntimeError('horizontal_control_mode must be config, velocity or attitude')
    parameters['bridge.horizontal_control_mode']=mode
    # Share the controller's speed ceiling instead of silently rejecting its
    # commands above the legacy pocket_bench.yaml bridge limit (1 m/s).
    limit = control['position_controller_node']['ros__parameters']['max_speed_mps']
    if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit <= 0:
        raise RuntimeError('Control parameter max_speed_mps must be a positive finite number')
    parameters['bridge.max_body_speed_mps'] = float(limit)
    rate = value('velocity_rate_hz')
    if rate.lower() == 'config':
        rate = parameters.get('bridge.velocity_rate_hz', 10.0)
    if isinstance(rate, bool):
        raise RuntimeError('velocity_rate_hz must be a finite number in 1..100')
    try:
        rate = float(rate)
    except (TypeError, ValueError):
        raise RuntimeError('velocity_rate_hz must be config or a finite number in 1..100')
    if not math.isfinite(rate) or not 1 <= rate <= 100:
        raise RuntimeError('velocity_rate_hz must be a finite number in 1..100')
    parameters.update({
        'manual.verify_device': flag('verify_device'),
        'bridge.tx_enabled': flag('tx_enabled'),
        'bridge.send_velocity': flag('send_velocity'),
        'bridge.velocity_rate_hz': rate,
        'bridge.send_rc': use_rc,
        'bridge.print_rx': False,
        'bridge.remote_ip': value('remote_ip'),
        'bridge.bind_ip': value('bind_ip'),
        'bridge.rx_port': int(value('rx_port')),
        'bridge.tx_port': int(value('tx_port')),
        'bridge.velocity_xy_only': True,
        'bridge.heading_xy_profile_confirmed': True,
        'bridge.velocity_frame_id': 'base_link_heading_frd',
    })
    if not use_rc:
        # Bridge only: no joystick acquisition, channel mapper or RC subscriber.
        bridge_parameters = {k[len('bridge.'):]: v for k, v in parameters.items()
                             if k.startswith('bridge.')}
        return [Node(package='flapping_uav_gcs', executable='mavlink_bridge_node',
                     name='mavlink_bridge_node', output='screen', parameters=[bridge_parameters])]
    return [Node(package='flapping_uav_gcs', executable='communication_node',
                 name='communication_node', output='screen', parameters=[parameters])]


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    defaults = {
        'config': os.path.join(share, 'config', 'pocket_bench.yaml'),
        'control_config': os.path.join(share, 'config', 'position_hold.yaml'),
        'tx_enabled': 'true', 'send_velocity': 'true',
        'velocity_rate_hz': 'config',
        'velocity_protocol': 'config',
        'horizontal_control_mode': 'config',
        'use_rc': 'true', 'use_joy': 'true', 'verify_device': 'true',
        'bind_ip': '0.0.0.0',
        'remote_ip': '192.168.1.151', 'rx_port': '9000', 'tx_port': '9111',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=nodes)])
