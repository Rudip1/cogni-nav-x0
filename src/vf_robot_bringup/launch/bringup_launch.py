#!/usr/bin/env python3

# =============================================================================
# vf_robot_bringup / bringup_launch.py
# =============================================================================
# THE single entry point for ViroFighter navigation.
#
# Usage (pick ONE mode):
#
#   # Mode 1 — RTAB-Map SLAM (build a new map)
#   ros2 launch vf_robot_bringup bringup_launch.py mode:=rtabmap_slam camera:=dual map_name:=my_office
#
#   # Mode 2 — RTAB-Map Localization (navigate in existing map)
#   ros2 launch vf_robot_bringup bringup_launch.py mode:=rtabmap_loc camera:=dual map_name:=my_office
#
#   # Mode 3 — AMCL (navigate in 2D map)
#   ros2 launch vf_robot_bringup bringup_launch.py mode:=amcl camera:=dual \
#       map:=$HOME/cogni-nav-x0/maps/my_office/my_office.yaml
#
#   # Mode 4 — SLAM Toolbox (build map with laser-based SLAM)
#   ros2 launch vf_robot_bringup bringup_launch.py mode:=slam_toolbox camera:=dual
#
#   # Real robot — any mode:
#   ros2 launch vf_robot_bringup bringup_launch.py mode:=rtabmap_loc ... use_sim_time:=false
#
# Prerequisites:
#   Gazebo (sim) or real robot driver must already be running.
#   e.g.  ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Package paths ──
    pkg_bringup = get_package_share_directory("vf_robot_bringup")
    pkg_slam = get_package_share_directory("vf_robot_slam")

    # ══════════════════════════════════════════════════════════════
    # DECLARE ARGUMENTS
    # ══════════════════════════════════════════════════════════════

    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="rtabmap_slam",
        choices=["rtabmap_slam", "rtabmap_loc", "amcl", "slam_toolbox"],
        description="Navigation mode: rtabmap_slam | rtabmap_loc | amcl | slam_toolbox",
    )

    declare_camera = DeclareLaunchArgument(
        "camera",
        default_value="dual",
        choices=["d435i", "d455", "dual"],
        description="Camera configuration for depth_to_scan and RTAB-Map",
    )

    declare_scan_method = DeclareLaunchArgument(
        "scan_method",
        default_value="pc2scan",
        choices=["dimg", "pc2scan"],
        description="Depth-to-scan conversion method (pc2scan recommended)",
    )

    declare_merge_scans = DeclareLaunchArgument(
        "merge_scans",
        default_value="true",
        choices=["true", "false"],
        description="Merge dual camera scans into single /scan topic",
    )

    declare_map_name = DeclareLaunchArgument(
        "map_name",
        default_value="default_map",
        description="Map folder name for RTAB-Map modes (.db file)",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to .yaml map file (AMCL mode only)",
    )

    declare_maps_dir = DeclareLaunchArgument(
        "maps_dir",
        default_value=os.path.join(os.path.expanduser("~"), "cogni-nav-x0", "maps"),
        description="Base directory for all maps",
    )

    declare_new_map = DeclareLaunchArgument(
        "new_map",
        default_value="true",
        choices=["true", "false"],
        description="RTAB-Map SLAM: delete existing .db and start fresh",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_bringup, "config", "nav2_params.yaml"),
        description="Full path to Nav2 parameters file",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo simulation clock (true) or wall clock (false)",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz with Nav2 panels",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_bringup, "rviz", "vf_bringup.rviz"),
        description="Full path to RViz config file",
    )

    # ══════════════════════════════════════════════════════════════
    # DEPTH TO SCAN — always runs (all 4 modes need /scan)
    # ══════════════════════════════════════════════════════════════

    depth_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, "launch", "depth_to_scan.launch.py")
        ),
        launch_arguments={
            "method": LaunchConfiguration("scan_method"),
            "camera": LaunchConfiguration("camera"),
            "merge_scans": LaunchConfiguration("merge_scans"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ══════════════════════════════════════════════════════════════
    # MODE 1 — RTAB-Map SLAM
    # ══════════════════════════════════════════════════════════════

    rtabmap_slam = GroupAction(
        condition=LaunchConfigurationEquals("mode", "rtabmap_slam"),
        actions=[
            LogInfo(msg="[vf_bringup] Mode: RTAB-Map SLAM — building new map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, "launch", "rtabmap_slam.launch.py")
                ),
                launch_arguments={
                    "camera": LaunchConfiguration("camera"),
                    "map_name": LaunchConfiguration("map_name"),
                    "maps_dir": LaunchConfiguration("maps_dir"),
                    "new_map": LaunchConfiguration("new_map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz": "false",  # bringup handles its own RViz
                }.items(),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════
    # MODE 2 — RTAB-Map Localization
    # ══════════════════════════════════════════════════════════════

    rtabmap_loc = GroupAction(
        condition=LaunchConfigurationEquals("mode", "rtabmap_loc"),
        actions=[
            LogInfo(
                msg="[vf_bringup] Mode: RTAB-Map Localization — loading existing map"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, "launch", "rtabmap_loc.launch.py")
                ),
                launch_arguments={
                    "camera": LaunchConfiguration("camera"),
                    "map_name": LaunchConfiguration("map_name"),
                    "maps_dir": LaunchConfiguration("maps_dir"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz": "false",
                }.items(),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════
    # MODE 3 — AMCL + map_server
    # ══════════════════════════════════════════════════════════════

    amcl_loc = GroupAction(
        condition=LaunchConfigurationEquals("mode", "amcl"),
        actions=[
            LogInfo(msg="[vf_bringup] Mode: AMCL — localization with 2D map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "localization_launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════
    # MODE 4 — SLAM Toolbox
    # ══════════════════════════════════════════════════════════════

    slam_toolbox = GroupAction(
        condition=LaunchConfigurationEquals("mode", "slam_toolbox"),
        actions=[
            LogInfo(
                msg="[vf_bringup] Mode: SLAM Toolbox — building map with laser SLAM"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "slam_launch.py")
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════
    # NAV2 STACK — always runs (planners, controllers, costmaps)
    # ══════════════════════════════════════════════════════════════

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ══════════════════════════════════════════════════════════════
    # RVIZ — optional
    # ══════════════════════════════════════════════════════════════

    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration("rviz")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "rviz_launch.py")
                ),
                launch_arguments={
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════
    # ASSEMBLE
    # ══════════════════════════════════════════════════════════════

    return LaunchDescription(
        [
            # Arguments
            declare_mode,
            declare_camera,
            declare_scan_method,
            declare_merge_scans,
            declare_map_name,
            declare_map,
            declare_maps_dir,
            declare_new_map,
            declare_params_file,
            declare_use_sim_time,
            declare_rviz,
            declare_rviz_config,
            # Depth to scan (all modes)
            depth_to_scan,
            # Mode-specific map/localization provider (exactly ONE activates)
            rtabmap_slam,
            rtabmap_loc,
            amcl_loc,
            slam_toolbox,
            # Nav2 stack (all modes)
            navigation,
            # RViz (optional)
            rviz,
        ]
    )
