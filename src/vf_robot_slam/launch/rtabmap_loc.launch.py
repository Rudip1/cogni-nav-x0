#!/usr/bin/env python3
"""
RTAB-Map Localization Launch File for ViroFighter UVC-1
═══════════════════════════════════════════════════════

Localizes within a previously built map using RTAB-Map visual localization.
Requires a .db file from a prior SLAM session.

Usage:
    # Gazebo
    ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office

    # Real robot
    ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office use_sim_time:=false

    # Custom maps directory
    ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_office maps_dir:=~/my_maps

Map must exist at: ~/cogni-nav-x0/maps/<map_name>/<map_name>.db
Build it first with:
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_rtabmap_loc_params(database_path, use_sim_time):
    """All RTAB-Map parameters for Localization mode — inline, no YAML files."""
    return {
        # ── Time (CRITICAL for Gazebo) ──
        "use_sim_time": use_sim_time,
        # ── Frames ──
        "frame_id": "base_footprint",   # must match odom child_frame_id
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "publish_tf": True,
        # ── Sync ──
        "queue_size": 30,
        "approx_sync": True,
        # ── Database ──
        "database_path": database_path,
        # ── Localization mode — do not add new nodes to the map ──
        "Mem/IncrementalMemory": "false",
        "Mem/InitWMWithAllNodes": "true",
        # ── Optimizer ──
        "Optimizer/Strategy": "1",
        "Optimizer/Iterations": "20",
        "Optimizer/Slam2D": "true",
        # ── Registration ──
        "Reg/Strategy": "0",
        "Reg/Force3DoF": "true",
        # ── Visual features ──
        "Vis/MinInliers": "15",
        "Vis/InlierDistance": "0.1",
        "Vis/MaxFeatures": "500",
        "Vis/FeatureType": "8",
        # ── Loop closure — more sensitive for reliable re-localization ──
        "Rtabmap/DetectionRate": "2.0",
        "Rtabmap/LoopThr": "0.09",
        "RGBD/LoopClosureReextractFeatures": "true",
        "RGBD/OptimizeMaxError": "3.0",
        # ── Map is static — do not update ──
        "RGBD/LinearUpdate": "0.0",
        "RGBD/AngularUpdate": "0.0",
        # ── Memory — keep working memory small ──
        "Mem/ImageKept": "false",
        "Mem/STMSize": "10",
        # ── Optimize from last known position ──
        "RGBD/OptimizeFromGraphEnd": "true",
    }


def launch_setup(context, *args, **kwargs):
    camera = LaunchConfiguration("camera").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    maps_dir = LaunchConfiguration("maps_dir").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context)

    use_sim_time = sim_time.lower() == "true"

    maps_dir = os.path.expanduser(maps_dir)
    map_folder = os.path.join(maps_dir, map_name)
    database_path = os.path.join(map_folder, f"{map_name}.db")

    pkg_share = get_package_share_directory("vf_robot_slam")

    actions = []

    # ── Guard: check database exists ─────────────────────────────────────────
    if not os.path.exists(database_path):
        actions.append(
            LogInfo(msg=[
                "\n", "=" * 70, "\n",
                "ERROR: Map database not found!\n",
                "=" * 70, "\n",
                f"Expected:  {database_path}\n",
                "\nBuild the map first:\n",
                f"  ros2 launch vf_robot_slam rtabmap_slam.launch.py "
                f"camera:={camera} map_name:={map_name}\n",
                "=" * 70, "\n",
            ])
        )
        return actions

    actions.append(
        LogInfo(msg=[
            "\n", "=" * 70, "\n",
            "RTAB-Map Localization Mode\n",
            "=" * 70, "\n",
            f"Camera:        {camera}\n",
            f"Map name:      {map_name}\n",
            f"Database:      {database_path}\n",
            f"Sim time:      {use_sim_time}\n",
            f"Frame ID:      base_footprint\n",
            "=" * 70, "\n",
        ])
    )

    # ── Include rgbd_sync (shared module) ────────────────────────────────────
    if camera == "dual":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "include", "rgbd_sync.launch.py")
                ),
                launch_arguments={
                    "camera": camera,
                    "use_sim_time": sim_time,
                }.items(),
            )
        )

    # ── RTAB-Map node ────────────────────────────────────────────────────────
    rtabmap_params = _get_rtabmap_loc_params(database_path, use_sim_time)

    if camera == "dual":
        rtabmap_params.update({
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_rgbd": True,
            "rgbd_cameras": 2,
        })
        remappings = [
            ("rgbd_image0", "/rgbd_image/d435i"),
            ("rgbd_image1", "/rgbd_image/d455"),
            ("odom", "/odom"),
            ("map", "/map"),
        ]
    else:
        # Single camera — subscribe directly
        rtabmap_params.update({
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_rgbd": False,
        })
        if camera == "d435i":
            remappings = [
                ("rgb/image", "/d435i/rgb/d435i_rgb/image_raw"),
                ("rgb/camera_info", "/d435i/rgb/d435i_rgb/camera_info"),
                ("depth/image", "/d435i/depth/d435i_depth/depth/image_raw"),
                ("depth/camera_info", "/d435i/depth/d435i_depth/depth/camera_info"),
                ("odom", "/odom"),
                ("map", "/map"),
            ]
        else:  # d455
            remappings = [
                ("rgb/image", "/d455/rgb/d455_rgb/image_raw"),
                ("rgb/camera_info", "/d455/rgb/d455_rgb/camera_info"),
                ("depth/image", "/d455/depth/d455_depth/depth/image_raw"),
                ("depth/camera_info", "/d455/depth/d455_depth/depth/camera_info"),
                ("odom", "/odom"),
                ("map", "/map"),
            ]

    actions.append(
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[rtabmap_params],
            remappings=remappings,
        )
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    if rviz.lower() == "true":
        rviz_config = os.path.join(pkg_share, "rviz", "rtabmap_loc.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera", default_value="dual", choices=["d435i", "d455", "dual"],
                description="Camera configuration.",
            ),
            DeclareLaunchArgument(
                "map_name", default_value="",
                description="Name of the map to load (REQUIRED).",
            ),
            DeclareLaunchArgument(
                "maps_dir", default_value="~/cogni-nav-x0/maps",
                description="Base directory containing map folders.",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", choices=["true", "false"],
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", choices=["true", "false"],
                description="true for Gazebo, false for real robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
