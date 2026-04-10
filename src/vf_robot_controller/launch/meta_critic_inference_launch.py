"""
meta_critic_inference_launch.py — META_CRITIC sidecar (Mode 3)

Launches ONLY the Python nodes needed alongside vf_robot_controller running
in INFERENCE mode. Does NOT spawn controller_server — that lives in
vf_robot_bringup/launch/bringup_launch.py with controller:=vf_inference.

Sidecar nodes:
  1. feature_extractor.py          — builds 410-dim feature vector from
                                     /local_costmap/costmap, /odom, /plan
  2. meta_critic_inference_node.py — runs meta_critic.pt and publishes
                                     critic weights on /vf_controller/meta_weights

Usage:
  # Terminal 1 — bringup with the inference-mode controller fragment
  ros2 launch vf_robot_bringup bringup_launch.py \\
       robot:=virofighter controller:=vf_inference localization:=rtabmap_loc

  # Terminal 2 — this sidecar (in conda dl env)
  conda activate dl
  ros2 launch vf_robot_controller meta_critic_inference_launch.py

Requires meta_critic/models/meta_critic.pt to exist
(produced by `python training/train.py --method META_CRITIC`).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vf_robot_controller')

    default_model = os.path.join(pkg, 'meta_critic', 'models', 'meta_critic.pt')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='Absolute path to the trained meta_critic.pt TorchScript model')

    feature_extractor = Node(
        package='vf_robot_controller',
        executable='feature_extractor.py',
        name='feature_extractor',
        output='screen',
    )

    inference_node = Node(
        package='vf_robot_controller',
        executable='meta_critic_inference_node.py',
        name='meta_critic_inference',
        output='screen',
        parameters=[
            {'model_path':  LaunchConfiguration('model_path')},
            {'num_critics': 10},
            {'feature_dim': 410},
        ],
    )

    return LaunchDescription([
        model_path_arg,
        feature_extractor,
        inference_node,
    ])
