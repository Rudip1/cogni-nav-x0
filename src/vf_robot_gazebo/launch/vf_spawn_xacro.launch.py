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
# Spawns the robot by reading /robot_description topic published
# by robot_state_publisher (xacro processed from vf_robot_description).
# Must be launched AFTER vf_robot_state_publisher.launch.py.
#
# Run:  ros2 launch vf_robot_gazebo vf_spawn_xacro.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    theta  = LaunchConfiguration('theta',  default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X spawn position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y spawn position'),
        DeclareLaunchArgument('theta',  default_value='0.0', description='Yaw spawn angle (radians)'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'vf_virofighter',
                '-topic', 'robot_description',  # reads /robot_description from RSP
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.1',
                '-Y', theta,
            ],
            output='screen',
        ),
    ])
