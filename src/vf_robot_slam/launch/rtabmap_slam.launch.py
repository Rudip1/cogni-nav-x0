#!/usr/bin/env python3
"""
RTAB-Map SLAM Launch File for ViroFighter UVC-1
════════════════════════════════════════════════

Builds a new map using RTAB-Map visual SLAM with Intel RealSense cameras.

Usage:
    # New map with both cameras (Gazebo)
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

    # Continue existing map
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=false

    # Single camera
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=d455 map_name:=my_office

    # Real robot
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office use_sim_time:=false

On shutdown (Ctrl+C):
    • my_office.db is saved automatically by RTAB-Map
    • Save 2D map manually while running:
        ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office

Critical lessons (from debugging):
    1. use_sim_time MUST be true for Gazebo — sim timestamps are ~1000 s,
       wall time is ~1.77 billion. Mismatch drops every RGBD frame silently.
    2. frame_id MUST be base_footprint — Gazebo publishes odom→base_footprint,
       NOT odom→base_link. Wrong frame causes silent TF lookup failures.
    3. depth/camera_info MUST be remapped in rgbd_sync — without it the
       RGBDImage messages have empty frame_ids.
    4. approx_sync_max_interval: 0.05 — 0.0 defeats approx_sync.
"""

import os
from pathlib import Path
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


def _get_rtabmap_slam_params(database_path, delete_db, use_sim_time):
    """All RTAB-Map parameters for SLAM mode — inline, no YAML files."""
    return {
        # ── Time (CRITICAL for Gazebo) ──
        "use_sim_time": use_sim_time,
        # ── Frames ──
        "frame_id": "base_footprint",  # must match odom child_frame_id
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "publish_tf": True,
        # ── Sync ──
        "queue_size": 30,
        "approx_sync": True,
        # ── Database ──
        "database_path": database_path,
        "delete_db_on_start": delete_db,
        # ── SLAM mode ──
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        # ── Registration — ICP avoids needing OpenGV for multi-camera ──
        "Reg/Strategy": "1",  # 1=ICP
        "Vis/EstimationType": "0",  # 0=3D-3D (not PnP, which needs OpenGV)
        "Reg/Force3DoF": "true",  # 2D SLAM for ground robot
        # ── Optimizer ──
        "Optimizer/Strategy": "1",
        "Optimizer/Iterations": "20",
        "Optimizer/Slam2D": "true",
        # ── Visual features ──
        "Vis/MinInliers": "15",
        "Vis/InlierDistance": "0.1",
        "Vis/MaxFeatures": "500",
        "Vis/FeatureType": "8",
        # ── Loop closure ──
        "Rtabmap/DetectionRate": "1.0",
        "Rtabmap/TimeThr": "0.0",
        "Rtabmap/LoopThr": "0.11",
        "RGBD/LoopClosureReextractFeatures": "true",
        "RGBD/OptimizeMaxError": "1.0",        # was 3.0 — tighten to reject bad loop closures during mapping
        # ── Mapping thresholds ──
        "RGBD/LinearUpdate": "0.1",
        "RGBD/AngularUpdate": "0.1",
        "RGBD/CreateOccupancyGrid": "true",
        # ── Memory ──
        "Mem/ImageKept": "true",
        "Mem/STMSize": "30",
        # ── Grid map ──
        "Grid/FromDepth": "true",
        "Grid/RayTracing": "true",
        "Grid/3D": "false",
        "Grid/CellSize": "0.05",
        "Grid/RangeMin": "0.0",
        "Grid/RangeMax": "6.0",
        "Grid/MaxGroundHeight": "0.05",
        "Grid/MaxObstacleHeight": "2.0",
    }


def launch_setup(context, *args, **kwargs):
    camera = LaunchConfiguration("camera").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    maps_dir = LaunchConfiguration("maps_dir").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    new_map = LaunchConfiguration("new_map").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context)

    use_sim_time = sim_time.lower() == "true"

    maps_dir = os.path.expanduser(maps_dir)
    map_folder = os.path.join(maps_dir, map_name)
    database_path = os.path.join(map_folder, f"{map_name}.db")

    Path(map_folder).mkdir(parents=True, exist_ok=True)

    pkg_share = get_package_share_directory("vf_robot_slam")

    delete_db = new_map.lower() == "true"
    mode_str = "NEW MAP (deleting existing)" if delete_db else "CONTINUING existing map"

    actions = []

    actions.append(
        LogInfo(
            msg=[
                "\n",
                "=" * 70,
                "\n",
                "RTAB-Map SLAM Mode\n",
                "=" * 70,
                "\n",
                f"Camera:        {camera}\n",
                f"Map name:      {map_name}\n",
                f"Map folder:    {map_folder}\n",
                f"Mode:          {mode_str}\n",
                f"Sim time:      {use_sim_time}\n",
                f"Frame ID:      base_footprint\n",
                "\n",
                "Save 2D map while running:\n",
                f"  ros2 run nav2_map_server map_saver_cli -f {map_folder}/{map_name}\n",
                "=" * 70,
                "\n",
            ]
        )
    )

    # ── Include rgbd_sync (shared module — no more copy-paste) ───────────────
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
    rtabmap_params = _get_rtabmap_slam_params(database_path, delete_db, use_sim_time)

    if camera == "dual":
        rtabmap_params.update(
            {
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_rgbd": True,
                "rgbd_cameras": 2,
            }
        )
        remappings = [
            ("rgbd_image0", "/rgbd_image/d435i"),
            ("rgbd_image1", "/rgbd_image/d455"),
            ("odom", "/odom"),
            ("map", "/map"),
        ]
    else:
        # Single camera — subscribe directly, no rgbd_sync needed
        rtabmap_params.update(
            {
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_rgbd": False,
            }
        )
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
        rviz_config = os.path.join(pkg_share, "rviz", "rtabmap_slam.rviz")
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
                "camera",
                default_value="dual",
                choices=["d435i", "d455", "dual"],
                description="Camera configuration.",
            ),
            DeclareLaunchArgument(
                "map_name",
                default_value="default_map",
                description="Name for the map folder and .db file.",
            ),
            DeclareLaunchArgument(
                "maps_dir",
                default_value="~/cogni-nav-x0/maps",
                description="Base directory where map folders are created.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                choices=["true", "false"],
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "new_map",
                default_value="true",
                choices=["true", "false"],
                description="Start fresh map (true) or continue existing (false).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
                description="true for Gazebo, false for real robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
