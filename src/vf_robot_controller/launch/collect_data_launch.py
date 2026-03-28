
"""
collect_data_launch.py — Mode 2: COLLECT

Launches:
  1. Nav2 controller server  (C++ plugin, mode=collect)
  2. feature_extractor.py    (builds 410-dim feature vector)
  3. data_logger.py          (writes HDF5 training files)

Usage:
  ros2 launch vf_robot_controller collect_data_launch.py
  ros2 launch vf_robot_controller collect_data_launch.py config:=hospital_params.yaml

Then navigate the robot in RViz. HDF5 files will accumulate in training/data/.
Stop when you have enough episodes (target: 300+ per map variant).
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

    # ── Nav2 controller server (C++ plugin in COLLECT mode) ───────────────────
    controller_server = Node(
        package    = 'nav2_controller',
        executable = 'controller_server',
        name       = 'controller_server',
        output     = 'screen',
        parameters = [
            config_file,
            {'VFController.controller_mode': 'collect'},
        ],
        remappings = [
            ('/cmd_vel', '/cmd_vel'),
        ],
    )

    # ── Feature extractor — builds 410-dim vector from sensors ────────────────
    feature_extractor = Node(
        package    = 'vf_robot_controller',
        executable = 'feature_extractor.py',
        name       = 'feature_extractor',
        output     = 'screen',
    )

    # ── Data logger — writes HDF5 files ───────────────────────────────────────
    data_logger = Node(
        package    = 'vf_robot_controller',
        executable = 'data_logger.py',
        name       = 'data_logger',
        output     = 'screen',
        parameters = [
            {'data_dir': 'training/data'},
        ],
    )

    return LaunchDescription([
        config_arg,
        controller_server,
        feature_extractor,
        data_logger,
    ])
