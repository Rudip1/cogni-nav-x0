"""
imitation_inference_launch.py — live imitation model deployment

Launches:
  1. Nav2 controller server  (vf_robot_controller in PASSIVE mode)
  2. feature_extractor.py    (builds 410-dim feature vector)
  3. imitation_inference_node.py  (runs imitation_best.pt → /cmd_vel)

Usage:
  ros2 launch vf_robot_controller imitation_inference_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("vf_robot_controller")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value="tb3_waffle_params.yaml",
        description="YAML config file from config/ directory",
    )

    config_file = os.path.join(pkg, "config", LaunchConfiguration("config"))

    model_path = os.path.join(pkg, "meta_critic", "models", "imitation.pt")

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            config_file,
            {"VFController.controller_mode": "passive"},
        ],
        remappings=[("/cmd_vel", "/cmd_vel")],
    )

    feature_extractor = Node(
        package="vf_robot_controller",
        executable="feature_extractor.py",
        name="feature_extractor",
        output="screen",
    )

    inference_node = Node(
        package="vf_robot_controller",
        executable="imitation_inference_node.py",
        name="imitation_inference",
        output="screen",
        parameters=[
            {"model_path": model_path},
            {"feature_dim": 410},
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            controller_server,
            feature_extractor,
            inference_node,
        ]
    )
