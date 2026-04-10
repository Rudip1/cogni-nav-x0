"""
imitation_inference_launch.py — IMITATION sidecar (PASSIVE mode deployment)

Launches ONLY the Python nodes needed alongside vf_robot_controller running
in PASSIVE mode. Does NOT spawn controller_server — that lives in
vf_robot_bringup/launch/bringup_launch.py with controller:=vf_passive.

In PASSIVE mode the C++ plugin's computeVelocityCommands() returns zero
immediately, so the Nav2 action server is satisfied but no /cmd_vel comes
out of C++. The Python imitation_inference_node owns /cmd_vel directly.

Sidecar nodes:
  1. feature_extractor.py         — builds 410-dim feature vector
  2. imitation_inference_node.py  — runs imitation.pt and publishes /cmd_vel

Usage:
  # Terminal 1 — bringup with the passive-mode controller fragment
  ros2 launch vf_robot_bringup bringup_launch.py \\
       robot:=virofighter controller:=vf_passive localization:=rtabmap_loc

  # Terminal 2 — this sidecar (in conda dl env)
  conda activate dl
  ros2 launch vf_robot_controller imitation_inference_launch.py

Requires meta_critic/models/imitation.pt to exist
(produced by `python training/train.py --method IMITATION`).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vf_robot_controller')

    default_model = os.path.join(pkg, 'meta_critic', 'models', 'imitation.pt')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='Absolute path to the trained imitation.pt TorchScript model')

    feature_extractor = Node(
        package='vf_robot_controller',
        executable='feature_extractor.py',
        name='feature_extractor',
        output='screen',
    )

    inference_node = Node(
        package='vf_robot_controller',
        executable='imitation_inference_node.py',
        name='imitation_inference',
        output='screen',
        parameters=[
            {'model_path':  LaunchConfiguration('model_path')},
            {'feature_dim': 410},
        ],
    )

    return LaunchDescription([
        model_path_arg,
        feature_extractor,
        inference_node,
    ])
