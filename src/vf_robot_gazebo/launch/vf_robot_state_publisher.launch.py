#!/usr/bin/env python3
#
# Copyright  EUROKNOWS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Pravin Oli
# https://www.euroknows.com/en/home/
#
# Publishes /robot_description from the xacro single source of truth in
# vf_robot_description. Both SDF-mode and xacro-mode Gazebo launches include
# this file — it provides the TF tree that RViz needs.
#
# Why xacro not cat+urdf:
#   uvc1_virofighter.xacro is the single source of truth (includes
#   sensors.xacro + common_properties.xacro). Using xacro here keeps
#   vf_robot_gazebo in sync with vf_robot_description automatically —
#   no manual conversion step needed for TF tree publishing.
#
# Used by:
#   vf_empty_world_xacro.launch.py    ← xacro mode: RSP provides /robot_description for spawn
#   vf_hospital_world_xacro.launch.py
#   vf_my_world_xacro.launch.py
#   vf_empty_world_sdf.launch.py      ← SDF mode: RSP needed only for TF tree (RViz)
#   vf_hospital_world_sdf.launch.py
#   vf_my_world_sdf.launch.py
#
# Run standalone:
#   ros2 launch vf_robot_gazebo vf_robot_state_publisher.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # xacro path — urdf/xacro/ subfolder layout
    xacro_path = PathJoinSubstitution([
        FindPackageShare('vf_robot_description'),
        'urdf', 'xacro',
        'uvc1_virofighter.xacro',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ParameterValue with value_type=str prevents ROS2 Humble from trying
    # to parse the URDF XML output as YAML (which causes a launch crash).
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_path,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description_content,
            }],
        ),
    ])
