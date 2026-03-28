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
# Hospital world — robot loaded from vf_robot_description xacro (single source of truth).
#
# Node graph:
#   gzserver                  → Gazebo physics server
#   gzclient                  → Gazebo GUI (delayed 3s — race condition fix)
#   robot_state_publisher     → processes xacro, publishes /robot_description + TF
#   spawn_entity              → reads /robot_description topic, spawns robot (delayed 5s)
#   rqt_robot_steering        → teleop GUI
#   rviz2                     → visualisation
#
# CRITICAL FIX — Environment Variables:
#   SetEnvironmentVariable MUST PREPEND to existing paths, never replace.
#   Replacing destroys Gazebo's own resource paths (/usr/share/gazebo-11),
#   causing RTShaderSystem errors and gzserver exit code 255.
#
# WHY TWO DELAYS:
#   gzclient delay (3s) — prevents "Assertion px != 0" GUI race crash.
#   spawn delay (5s)    — prevents gzserver crash when loading package://
#                         mesh URIs before the rendering scene is ready.
#
# Run:  ros2 launch vf_robot_gazebo vf_hospital_world_xacro.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vf_gazebo = get_package_share_directory("vf_robot_gazebo")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_desc = get_package_share_directory("vf_robot_description")
    launch_dir = os.path.join(pkg_vf_gazebo, "launch")

    # ── Launch arguments ───────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )
    declare_x_pose = DeclareLaunchArgument(
        "x_pose",
        default_value="-2.0",
        description="X spawn position",
    )
    declare_y_pose = DeclareLaunchArgument(
        "y_pose",
        default_value="-0.5",
        description="Y spawn position",
    )
    declare_theta = DeclareLaunchArgument(
        "theta",
        default_value="0.0",
        description="Yaw spawn angle (radians)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    theta = LaunchConfiguration("theta")

    world = os.path.join(pkg_vf_gazebo, "worlds", "hospital.world")
    rviz_config = os.path.join(pkg_vf_gazebo, "rviz", "vf_robot_gazebo.rviz")

    # ── Environment variables — PREPEND, never replace ────────────────────
    # Hardcode system Gazebo paths to ensure they're always included.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=os.pathsep.join(
            [
                os.path.dirname(pkg_desc),  # vf_robot_description share parent
                "/usr/share/gazebo-11",  # Gazebo Classic resources
                "/opt/ros/humble/share",  # ROS 2 Humble resources
            ]
        ),
    )
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join(
            [
                os.path.join(pkg_vf_gazebo, "models"),  # vf_robot_gazebo models
                os.path.dirname(pkg_desc),  # for package:// mesh resolution
                "/usr/share/gazebo-11/models",  # Gazebo default models
            ]
        ),
    )
    gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.pathsep.join(
            [
                "/opt/ros/humble/lib",  # ROS 2 Gazebo plugins
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",  # Gazebo plugins
            ]
        ),
    )

    # ── Gazebo server ──────────────────────────────────────────────────────
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    # ── Gazebo client — delayed 3s (prevents GUI race crash) ───────────────
    gzclient_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                )
            )
        ],
    )

    # ── Robot state publisher — processes xacro immediately ────────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "vf_robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Spawn robot — delayed 5s (prevents gzserver mesh-load crash) ───────
    # FIX: Was referencing non-existent vf_spawn_urdf.launch.py
    #      Correct file is vf_spawn_xacro.launch.py
    spawn_robot_cmd = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "vf_spawn_xacro.launch.py")
                ),
                launch_arguments={
                    "x_pose": x_pose,
                    "y_pose": y_pose,
                    "theta": theta,
                }.items(),
            )
        ],
    )

    # ── Teleop GUI ─────────────────────────────────────────────────────────
    gui_teleop_node = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        name="rqt_robot_steering",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── RViz — delayed 6s (after robot spawn) ──────────────────────────────
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    ld = LaunchDescription()

    # Declare arguments first
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_theta)

    # Environment variables MUST be set before gzserver starts
    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo_model_path)
    ld.add_action(gazebo_plugin_path)

    # Launch sequence
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)  # delayed 3s
    ld.add_action(robot_state_publisher_cmd)  # starts immediately
    ld.add_action(spawn_robot_cmd)  # delayed 5s
    ld.add_action(gui_teleop_node)
    ld.add_action(rviz_node)  # delayed 6s

    return ld
