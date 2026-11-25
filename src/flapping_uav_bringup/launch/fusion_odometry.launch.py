import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('flapping_uav_bringup')
    with open(LaunchConfiguration('config').perform(context), encoding='utf-8') as stream:
        parameters = dict(yaml.safe_load(stream)['fusion_odometry_node']['ros__parameters'])
    for key in ('publish_rate_hz', 'path_publish_rate_hz', 'alignment_max_clock_skew_ms', 'max_flow_speed_mps',
                'max_dead_reckoning_s', 'uwb_jump_slack_m', 'max_uwb_speed_mps',
                'uwb_innovation_gate_m', 'uwb_correction_gain', 'initial_position_variance_m2',
                'position_variance_growth_m2_per_s', 'uwb_sync_period_s',
                'uwb_recovery_confirmation_s', 'uwb_recovery_blend_s', 'uwb_filter_tau_s'):
        if key in parameters:
            value = parameters[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise RuntimeError('Fusion parameter {} must be a finite number'.format(key))
            parameters[key] = float(value)
    return [
        Node(package='flapping_uav_gcs', executable='fusion_odometry_node',
             name='fusion_odometry_node', parameters=[parameters], output='screen'),
        Node(package='rviz2', executable='rviz2', name='fusion_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'fusion_comparison.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'fusion_odometry.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
