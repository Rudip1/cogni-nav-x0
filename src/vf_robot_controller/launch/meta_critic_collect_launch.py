"""
meta_critic_collect_launch.py — META_CRITIC sidecar (Mode 2: data collection)

Launches ONLY the Python nodes needed alongside vf_robot_controller running
in COLLECT mode. Does NOT spawn controller_server — that lives in
vf_robot_bringup/launch/bringup_launch.py with controller:=vf_collect.

Sidecar nodes:
  1. feature_extractor.py         — builds 410-dim feature vector
  2. meta_critic_data_logger.py   — subscribes to /vf_controller/critic_data
                                    (published by the C++ DataRecorder) and
                                    writes HDF5 files containing the N×K
                                    critic-score matrix per timestep.

HDF5 files written to: training/data/run_*.h5

Usage:
  # Terminal 1 — bringup with the collect-mode controller fragment
  ros2 launch vf_robot_bringup bringup_launch.py \\
       robot:=virofighter controller:=vf_collect localization:=rtabmap_loc

  # Terminal 2 — this sidecar (in conda dl env)
  conda activate dl
  ros2 launch vf_robot_controller meta_critic_collect_launch.py

Then drive the robot via Nav2 Goal in RViz to gather episodes.
Target: 300+ episodes per map variant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='training/data',
        description='Directory to write META_CRITIC HDF5 files')

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
        parameters=[{'data_dir': LaunchConfiguration('data_dir')}],
    )

    return LaunchDescription([
        data_dir_arg,
        feature_extractor,
        data_logger,
    ])
