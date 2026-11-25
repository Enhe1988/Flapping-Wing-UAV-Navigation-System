"""Backup UDP transport, with joystick acquisition and RC forwarding disabled."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    defaults = {
        'config': os.path.join(share, 'config', 'pocket_bench.yaml'),
        'control_config': os.path.join(share, 'config', 'position_hold.yaml'),
        'bind_ip': '0.0.0.0', 'rx_port': '14550',
        # Backpack receives GCS uplink on 14555 and sends telemetry to 14550.
        # Confirmed AP layout: Backpack/gateway=.1, this computer=.100.
        # The destination is the Backpack, never this computer's own address.
        'remote_ip': '10.0.0.1', 'tx_port': '14555',
        'tx_enabled': 'true', 'send_velocity': 'true',
        'velocity_rate_hz': 'config',
        'velocity_protocol': 'config',
        'horizontal_control_mode': 'config',
    }
    arguments = {key: LaunchConfiguration(key) for key in defaults}
    arguments.update(use_rc='false', use_joy='false', verify_device='false')
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()] +
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'communication.launch.py')),
            launch_arguments=arguments.items())])
