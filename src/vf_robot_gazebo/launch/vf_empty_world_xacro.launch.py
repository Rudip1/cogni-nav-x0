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
# Empty world — robot loaded from vf_robot_description xacro (single source of truth).
#
# Node graph:
#   gzserver                  → Gazebo physics server
#   gzclient                  → Gazebo GUI (delayed 3s — race condition fix)
#   robot_state_publisher     → processes xacro, publishes /robot_description + TF
#   spawn_entity              → reads /robot_description topic, spawns robot (delayed 5s)
#   rqt_robot_steering        → teleop GUI
#
# WHY GAZEBO_RESOURCE_PATH:
#   Gazebo Classic cannot resolve package://vf_robot_description/meshes/...
#   on its own in ROS2. Setting GAZEBO_RESOURCE_PATH to the parent of
#   vf_robot_description's share dir lets Gazebo find the meshes via
#   package:// URI resolution.
#
# WHY TWO DELAYS:
#   gzclient delay (3s) — prevents "Assertion px != 0" GUI race crash.
#   spawn delay (5s)    — prevents gzserver crash when loading package:// mesh
#                         URIs before the rendering scene is fully initialised.
#                         model.sdf (model:// URIs) doesn't need this because
#                         Gazebo loads those from its model database differently.
#
# Run:  ros2 launch vf_robot_gazebo vf_empty_world_xacro.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vf_gazebo  = get_package_share_directory('vf_robot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_desc       = get_package_share_directory('vf_robot_description')
    launch_dir     = os.path.join(pkg_vf_gazebo, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose', default='0.0')
    y_pose       = LaunchConfiguration('y_pose', default='0.0')
    theta        = LaunchConfiguration('theta',  default='0.0')

    world = os.path.join(pkg_vf_gazebo, 'worlds', 'empty_world.world')

    # ── GAZEBO_RESOURCE_PATH — MUST be set before gzserver starts ─────────
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg_desc),
    )

    # ── Gazebo server ─────────────────────────────────────────────────────
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # ── Gazebo client — delayed 3s (prevents GUI race crash) ──────────────
    gzclient_cmd = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )],
    )

    # ── Robot state publisher — processes xacro immediately ───────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'vf_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Spawn robot — delayed 5s (prevents gzserver mesh-load crash) ──────
    # package:// URIs in xacro URDF require the Gazebo rendering scene to be
    # fully initialised before spawn_entity sends the robot description.
    # model.sdf uses model:// URIs which Gazebo loads differently (no crash).
    spawn_robot_cmd = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'vf_spawn_xacro.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'theta':  theta,
            }.items(),
        )],
    )

    # ── Teleop GUI ────────────────────────────────────────────────────────
    gui_teleop_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_resource_path)       # MUST be first — before gzserver
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)               # delayed 3s
    ld.add_action(robot_state_publisher_cmd)  # starts immediately
    ld.add_action(spawn_robot_cmd)            # delayed 5s
    ld.add_action(gui_teleop_node)
    return ld
