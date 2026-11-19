import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('flapping_uav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='normal',
                              description='normal, missing_tag, nonfinite, jump, '
                                          'duplicate, dropout, wrap_restart'),
        Node(package='flapping_uav_gcs', executable='uwb_position_node',
             name='uwb_position_node', output='screen',
             parameters=[os.path.join(share, 'config', 'uwb_demo.yaml')]),
        Node(package='flapping_uav_gcs', executable='uwb_simulator_node',
             name='uwb_simulator_node', output='screen',
             parameters=[{'tag_id': 0, 'rate_hz': 20.0,
                          'scenario': LaunchConfiguration('scenario')}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'uwb.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
