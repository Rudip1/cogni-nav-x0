#!/usr/bin/env python3

# =============================================================================
# vf_robot_bringup / localization_launch.py
# =============================================================================
# Mode 3: AMCL localization with a pre-built 2D occupancy grid map.
#
# Starts:
#   - map_server:  loads .pgm/.yaml map → publishes /map
#   - amcl:        subscribes to /scan + /map → publishes map→odom TF
#   - lifecycle_manager: manages both nodes
#
# Requires:
#   - A .yaml map file (from map_saver_cli or rtabmap export)
#   - /scan topic (from depth_to_scan.launch.py)
#   - /odom and odom→base_footprint TF (from Gazebo or robot driver)
#
# NOT launched directly — included by bringup_launch.py.
# =============================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments ──
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── map_server ──
    # Loads the 2D occupancy grid from .pgm/.yaml and publishes on /map.
    # This is the ONLY /map publisher in AMCL mode.

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            },
        ],
    )

    # ── AMCL ──
    # Adaptive Monte Carlo Localization.
    # Subscribes to /scan and /map, publishes map→odom TF.
    # User provides initial pose via RViz "2D Pose Estimate" button.

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── Lifecycle Manager ──
    # map_server must be active before AMCL (AMCL needs /map to initialize).

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Full path to map .yaml file"),
            DeclareLaunchArgument("params_file", description="Nav2 params YAML"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            map_server,
            amcl,
            lifecycle_manager_localization,
        ]
    )
