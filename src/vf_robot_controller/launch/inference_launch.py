"""
inference_launch.py — Mode 3: INFERENCE

Launches:
  1. Nav2 controller server  (C++ plugin, mode=inference)
  2. feature_extractor.py    (builds 410-dim feature vector)
  3. inference_node.py       (runs meta_critic.pt, publishes weights)

Usage:
  ros2 launch vf_robot_controller inference_launch.py
  ros2 launch vf_robot_controller inference_launch.py config:=hospital_params.yaml

Requires meta_critic/models/meta_critic.pt to exist (run train.py first).
If model is missing, inference_node falls back to uniform weights automatically
and the controller operates as classical MPPI — no crash, no manual intervention.
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

    model_path = os.path.join(
        pkg, 'meta_critic', 'models', 'meta_critic.pt')

    # ── Nav2 controller server (C++ plugin in INFERENCE mode) ─────────────────
    controller_server = Node(
        package    = 'nav2_controller',
        executable = 'controller_server',
        name       = 'controller_server',
        output     = 'screen',
        parameters = [
            config_file,
            {'VFController.controller_mode': 'inference'},
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

    # ── Inference node — loads model, publishes dynamic weights ───────────────
    inference_node = Node(
        package    = 'vf_robot_controller',
        executable = 'inference_node.py',
        name       = 'meta_critic_inference',
        output     = 'screen',
        parameters = [
            {'model_path':  model_path},
            {'num_critics': 10},
            {'feature_dim': 410},
        ],
    )

    return LaunchDescription([
        config_arg,
        controller_server,
        feature_extractor,
        inference_node,
    ])
