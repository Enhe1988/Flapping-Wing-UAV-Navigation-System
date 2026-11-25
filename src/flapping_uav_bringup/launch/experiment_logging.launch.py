import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def recorder(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    share = get_package_share_directory('flapping_uav_bringup')
    with open(value('logging_config'), encoding='utf-8') as stream:
        parameters = dict(yaml.safe_load(stream)['experiment_recorder_node']['ros__parameters'])
    parameters.update({
        'root_directory': os.path.abspath(value('log_root')),
        'experiment_label': value('experiment_label'),
        'flight_mode': value('flight_mode'),
        'exporter_script': os.path.join(get_package_prefix('flapping_uav_gcs'), 'lib', 'flapping_uav_gcs', 'flight_log.py'),
        'config_files': [value('control_config'), value('uwb_config'), value('scene_config'), value('logging_config'), value('localization_config'), value('trajectory_config')],
    })
    return [Node(package='flapping_uav_gcs', executable='experiment_recorder_node',
                 name='experiment_recorder_node', parameters=[parameters], output='screen',
                 sigterm_timeout='300', sigkill_timeout='5')]


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    defaults = {
        'logging_config': os.path.join(share, 'config', 'experiment_logging.yaml'),
        'control_config': os.path.join(share, 'config', 'position_hold.yaml'),
        'uwb_config': os.path.join(share, 'config', 'uwb.yaml'),
        'scene_config': os.path.join(share, 'config', 'scene.yaml'),
        'localization_config': os.path.join(share,'config','fusion_odometry.yaml'),
        'trajectory_config': os.path.join(share,'config','trajectory.yaml'),
        'log_root': os.path.join(os.getcwd(), 'log', 'experiments'),
        'experiment_label': 'flight',
        'flight_mode': 'waypoint',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=recorder)])
