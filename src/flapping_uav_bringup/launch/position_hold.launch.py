import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('flapping_uav_bringup')
    config_path = LaunchConfiguration('config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    parameters = dict(config['position_controller_node']['ros__parameters'])
    parameters.update(config['frame_contract'])
    # YAML treats "2" as an integer. These parameters are declared as double
    # by rclcpp, so normalize numeric scalars before Foxy creates the param file.
    for key in ('kp_xy', 'kd_xy', 'max_speed_mps', 'max_acceleration_mps2',
                'arrival_position_tolerance_m', 'arrival_speed_tolerance_mps',
                'arrival_dwell_s', 'publish_rate_hz', 'map_to_enu_yaw_rad',
                'alignment_max_clock_skew_ms'):
        if key not in parameters:
            continue
        number = parameters[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise RuntimeError('Control parameter {} must be a finite number'.format(key))
        parameters[key] = float(number)
    return [
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'experiment_logging.launch.py')),
            launch_arguments={
                'logging_config': LaunchConfiguration('logging_config'),
                'control_config': config_path,
                'uwb_config': LaunchConfiguration('uwb_config'),
                'scene_config': LaunchConfiguration('scene_config'),
                'log_root': LaunchConfiguration('log_root'),
                'experiment_label': LaunchConfiguration('experiment_label'),
            }.items())], condition=IfCondition(LaunchConfiguration('record_experiment'))),
        Node(package='flapping_uav_gcs', executable='position_controller_node',
             name='position_controller_node', output='screen', parameters=[parameters]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'uwb.launch.py')),
            launch_arguments={
                # Foxy inherits the parent's launch configurations, including
                # "config". Override it explicitly for the UWB child.
                'config': LaunchConfiguration('uwb_config'),
                'scene_config': LaunchConfiguration('scene_config'),
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'coordinates_confirmed': LaunchConfiguration('coordinates_confirmed'),
                'control_config': config_path,
                'use_scene': 'true', 'use_rviz': LaunchConfiguration('use_rviz'),
            }.items())], condition=IfCondition(LaunchConfiguration('start_uwb'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    defaults = {
        'config': os.path.join(share, 'config', 'position_hold.yaml'),
        'uwb_config': os.path.join(share, 'config', 'uwb.yaml'),
        'scene_config': os.path.join(share, 'config', 'scene.yaml'),
        'start_uwb': 'true', 'use_rviz': 'false',
        'record_experiment': 'true',
        'logging_config': os.path.join(share, 'config', 'experiment_logging.yaml'),
        'log_root': os.path.join(os.getcwd(), 'log', 'experiments'),
        'experiment_label': 'flight',
        'port_name': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        'baud_rate': '921600', 'coordinates_confirmed': 'true',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=nodes)])
