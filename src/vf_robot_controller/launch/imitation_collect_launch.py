"""
imitation_collect_launch.py — IMITATION data collection

Launches ONLY:
  1. feature_extractor.py        (builds 410-dim feature vector)
  2. imitation_data_logger.py    (records features + /cmd_vel from teacher)

Does NOT load vf_robot_controller. Run your system Nav2 separately with
any controller (DWB, MPPI, RPP). This node just listens to whatever
controller is running and records its /cmd_vel as the imitation label.

Usage:
  # Terminal 1 — start your system Nav2 with chosen teacher controller
  ros2 launch nav2_bringup navigation_launch.py params_file:=<your_nav2_params>

  # Terminal 2 — start imitation collection from this package
  ros2 launch vf_robot_controller imitation_collect_launch.py

HDF5 files written to: training/imitation/run_*.h5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='training/imitation',
        description='Directory to write imitation HDF5 files')

    feature_extractor = Node(
        package='vf_robot_controller',
        executable='feature_extractor.py',
        name='feature_extractor',
        output='screen',
    )

    imitation_logger = Node(
        package='vf_robot_controller',
        executable='imitation_data_logger.py',
        name='imitation_data_logger',
        output='screen',
        parameters=[{'data_dir': LaunchConfiguration('data_dir')}],
    )

    return LaunchDescription([
        data_dir_arg,
        feature_extractor,
        imitation_logger,
    ])
