import math
import importlib.util
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    compact=LaunchConfiguration('compact_yaw_only').perform(context).lower()
    if compact not in ('true','false','config'):
        raise RuntimeError('compact_yaw_only must be true, false or config')
    if compact!='config':
        parameters['compact_yaw_only']=compact=='true'
    mode=LaunchConfiguration('horizontal_control_mode').perform(context).lower()
    if mode not in ('config','velocity','attitude'):
        raise RuntimeError('horizontal_control_mode must be config, velocity or attitude')
    if mode!='config':parameters['horizontal_control_mode']=mode
    elif compact=='false':parameters['horizontal_control_mode']='velocity'
    # The localization node runs in both A/B modes. Use its accepted low-pass
    # UWB fixes even when optical-flow fallback is disabled; never filter twice.
    parameters['uwb_position_topic']='/uwb/position_filtered'
    fallback=LaunchConfiguration('use_optical_flow_fallback').perform(context).lower()
    if fallback not in ('true','false','config'):
        raise RuntimeError('use_optical_flow_fallback must be true, false or config')
    if fallback!='config':
        parameters['use_optical_flow_fallback']=fallback=='true'
    parameters.update(config['frame_contract'])
    mode=LaunchConfiguration('flight_mode').perform(context).lower()
    if mode not in ('waypoint','line','figure8'):
        raise RuntimeError('flight_mode must be waypoint, line or figure8')
    parameters['flight_mode']=mode
    if mode!='waypoint':
        with open(LaunchConfiguration('trajectory_config').perform(context),encoding='utf-8') as stream:
            trajectory=yaml.safe_load(stream)['trajectory']
        with open(LaunchConfiguration('scene_config').perform(context),encoding='utf-8') as stream:
            scene=yaml.safe_load(stream)['scene_visualizer_node']['ros__parameters']
        with open(LaunchConfiguration('uwb_config').perform(context),encoding='utf-8') as stream:
            uwb=yaml.safe_load(stream)['uwb_position_node']['ros__parameters']
        spec=importlib.util.spec_from_file_location('trajectory_geometry',os.path.join(os.path.dirname(__file__),'trajectory_geometry.py'))
        helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
        geometry=helper.geometry(scene,uwb,float(trajectory['boundary_margin_m']),float(trajectory['figure8_fit_scale']),
                                 float(trajectory['line_length_m']),trajectory.get('figure8_axis','x'),
                                 float(trajectory['figure8_lobe_aspect_ratio']) if 'figure8_lobe_aspect_ratio' in trajectory else None)
        period=float(LaunchConfiguration('trajectory_period_s').perform(context))
        if not math.isfinite(period) or period<0:raise RuntimeError('trajectory_period_s must be 0 (config) or positive')
        cycles=int(LaunchConfiguration('trajectory_cycles').perform(context))
        if cycles<0:raise RuntimeError('trajectory_cycles must be 0 (config) or positive')
        parameters.update({
            'trajectory_center_xy':[float(x) for x in geometry['center']],
            'trajectory_half_extents_xy':[float(x) for x in geometry['line_half_extents' if mode=='line' else 'figure8_half_extents']],
            # Avoid bare "y", which Foxy's C YAML parser treats as boolean.
            'trajectory_figure8_axis':'map_'+geometry['figure8_axis'],
            'trajectory_start_position_tolerance_m':float(trajectory.get('start_position_tolerance_m',parameters['arrival_position_tolerance_m'])),
            'trajectory_boundary_xy':[float(x) for x in geometry['polygon']],
            'trajectory_margin_m':float(trajectory['boundary_margin_m']),
            'trajectory_period_s':period or float(trajectory[mode+'_period_s']),
            'trajectory_cycles':cycles or int(trajectory['cycles']),
            'trajectory_ramp_s':float(trajectory['ramp_s']),
            'trajectory_max_speed_mps':min(float(trajectory['max_reference_speed_mps']),float(parameters['max_speed_mps'])),
            'trajectory_max_acceleration_mps2':min(float(trajectory['max_reference_acceleration_mps2']),float(parameters['max_acceleration_mps2'])),
            'trajectory_min_approach_s':float(trajectory['min_approach_time_s']),
        })
    # YAML treats "2" as an integer. These parameters are declared as double
    # by rclcpp, so normalize numeric scalars before Foxy creates the param file.
    for key in ('kp_xy', 'kd_xy', 'kp_x', 'kp_y', 'kd_x', 'kd_y', 'max_speed_mps', 'max_acceleration_mps2',
                'arrival_position_tolerance_m', 'arrival_speed_tolerance_mps',
                'arrival_dwell_s', 'publish_rate_hz', 'map_to_enu_yaw_rad',
                'alignment_max_clock_skew_ms','attitude_kp_x_deg_per_m','attitude_kp_y_deg_per_m',
                'attitude_ki_x_deg_per_m_s','attitude_ki_y_deg_per_m_s',
                'attitude_kd_x_deg_s_per_m','attitude_kd_y_deg_s_per_m',
                'attitude_integral_limit_deg','attitude_integral_zone_m','attitude_integral_max_speed_mps',
                'attitude_derivative_limit_deg','attitude_velocity_filter_tau_s',
                'attitude_velocity_limit_mps','attitude_sample_max_gap_s',
                'attitude_max_angle_deg','attitude_max_rate_deg_s'):
        if key not in parameters:
            continue
        number = parameters[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise RuntimeError('Control parameter {} must be a finite number'.format(key))
        parameters[key] = float(number)
    controller = Node(package='flapping_uav_gcs', executable='position_controller_node',
                      name='position_controller_node', output='screen', parameters=[parameters])
    return [
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share,'launch','fusion_odometry.launch.py')),
            launch_arguments={'config':LaunchConfiguration('localization_config'),'use_rviz':'false'}.items())]),
        RegisterEventHandler(OnProcessExit(target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='Navigation stopped; finalize paper CSV log'))])),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'experiment_logging.launch.py')),
            launch_arguments={
                'logging_config': LaunchConfiguration('logging_config'),
                'control_config': config_path,
                'uwb_config': LaunchConfiguration('uwb_config'),
                'scene_config': LaunchConfiguration('scene_config'),
                'localization_config': LaunchConfiguration('localization_config'),
                'trajectory_config': LaunchConfiguration('trajectory_config'),
                'log_root': LaunchConfiguration('log_root'),
                'experiment_label': LaunchConfiguration('experiment_label'),
                'flight_mode': LaunchConfiguration('flight_mode'),
            }.items())], condition=IfCondition(LaunchConfiguration('record_experiment'))),
        controller,
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'uwb.launch.py')),
            launch_arguments={
                # Foxy inherits the parent's launch configurations, including
                # "config". Override it explicitly for the UWB child.
                'config': LaunchConfiguration('uwb_config'),
                'scene_config': LaunchConfiguration('scene_config'),
                'scene_position_topic': LaunchConfiguration('scene_position_topic'),
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
        'localization_config': os.path.join(share,'config','fusion_odometry.yaml'),
        'trajectory_config': os.path.join(share,'config','trajectory.yaml'),
        'flight_mode':'waypoint', 'trajectory_period_s':'0', 'trajectory_cycles':'0',
        'use_optical_flow_fallback': 'config',
        'compact_yaw_only': 'config',
        'horizontal_control_mode': 'config',
        'start_uwb': 'true', 'use_rviz': 'false',
        # Aircraft model follows the SAME filtered observation used by UWB control.
        'scene_position_topic': '/uwb/position_filtered',
        'record_experiment': 'true',
        'logging_config': os.path.join(share, 'config', 'experiment_logging.yaml'),
        'log_root': os.path.join(os.getcwd(), 'log', 'experiments'),
        'experiment_label': 'flight',
        'port_name': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        'baud_rate': '921600', 'coordinates_confirmed': 'true',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=nodes)])
