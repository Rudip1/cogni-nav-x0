"""
meta_critic_collect_launch.py — Mode 2: META_CRITIC data collection

Launches:
  1. Nav2 controller server  (vf_robot_controller C++ plugin, mode=collect)
  2. feature_extractor.py    (builds 410-dim feature vector)
  3. meta_critic_data_logger.py  (writes HDF5 with critic score matrix)

Usage:
  ros2 launch vf_robot_controller meta_critic_collect_launch.py
  ros2 launch vf_robot_controller meta_critic_collect_launch.py config:=hospital_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vf_robot_controller')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='hospital_params.yaml',
        description='YAML config file from config/ directory')

    config_file = os.path.join(
        pkg, 'config',
        LaunchConfiguration('config'))

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            config_file,
            {'VFController.controller_mode': 'collect'},
        ],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    feature_extractor = Node(
        package='vf_robot_controller',
        executable='feature_extractor.py',
        name='feature_extractor',
        output='screen',
    )

    data_logger = Node(
        package='vf_robot_controller',
        executable='meta_critic_data_logger.py',
        name='meta_critic_data_logger',
        output='screen',
        parameters=[{'data_dir': 'training/data'}],
    )

    return LaunchDescription([
        config_arg,
        controller_server,
        feature_extractor,
        data_logger,
    ])
