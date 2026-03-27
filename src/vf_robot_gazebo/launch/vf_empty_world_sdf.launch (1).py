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
# Empty world — robot loaded from SDF inside vf_robot_gazebo/models/
#
# Node graph:
#   gzserver                  → Gazebo physics server
#   gzclient                  → Gazebo GUI (delayed 3s to avoid race crash)
#   vf_spawn_sdf              → spawns robot from model.sdf
#   vf_robot_state_publisher  → processes xacro, publishes TF tree (needed by RViz)
#   rqt_robot_steering        → teleop GUI
#
# WHY robot_state_publisher in SDF mode:
#   Gazebo diff drive publishes odom→base_footprint only.
#   RSP publishes base_footprint→base_link→all sensor/wheel frames.
#   Without RSP, RViz has no TF tree.
#
# WHY gzclient delay:
#   Gazebo Classic has a race condition where gzclient crashes with
#   "Assertion px != 0" if it starts before gzserver has initialised
#   the rendering scene. A 3-second TimerAction delay fixes this.
#
# Run:  ros2 launch vf_robot_gazebo vf_empty_world_sdf.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vf_gazebo  = get_package_share_directory('vf_robot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_dir     = os.path.join(pkg_vf_gazebo, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose', default='0.0')
    y_pose       = LaunchConfiguration('y_pose', default='0.0')
    theta        = LaunchConfiguration('theta',  default='0.0')

    world = os.path.join(pkg_vf_gazebo, 'worlds', 'empty_world.world')

    # ── Gazebo server ─────────────────────────────────────────────────────
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # ── Gazebo client (delayed 3s — avoids rendering scene race crash) ────
    gzclient_cmd = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )],
    )

    # ── Robot state publisher — processes xacro, publishes TF ─────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'vf_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Spawn robot from SDF file ─────────────────────────────────────────
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'vf_spawn_sdf.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'theta':  theta,
        }.items(),
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
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)               # delayed 3s
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(gui_teleop_node)
    return ld
