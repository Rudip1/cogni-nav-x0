#!/usr/bin/env python3
"""
Depth → LaserScan using pointcloud_to_laserscan — ViroFighter UVC-1
════════════════════════════════════════════════════════════════════════

This file is included by depth_to_scan.launch.py (method:=pc2scan).
Do NOT launch directly — use:
    ros2 launch vf_robot_slam depth_to_scan.launch.py method:=pc2scan camera:=dual

WHEN TO USE
───────────
  Real robot:
    • RealSense publishes PointCloud2 at full 15–30 Hz from onboard processor
    • No floor false-obstacles — height filter works in world-space Z
    • Obstacles at any distance detected correctly regardless of camera tilt
    • This is the correct method for production use

  Simulation (Gazebo):
    • Gazebo generates PointCloud2 in software → CPU bottleneck → ~2–3 Hz
    • Use method:=dimg for simulation instead

WHY THIS BEATS depthimage_to_laserscan FOR D435i
─────────────────────────────────────────────────
  depthimage_to_laserscan collapses image rows — scan plane locked to camera
  optical axis. D435i at 60° tilt: even top image row is 31° below horizontal.
  Floor always appears as obstacles. No fix exists within that node.

  pointcloud_to_laserscan filters in WORLD Z — tilt is irrelevant.
  Floor excluded cleanly. range_min can be 0.1 m (no range hack needed).

HOW IT WORKS
────────────
  1. Subscribes to PointCloud2 (full 3D point cloud from depth camera)
  2. Transforms every point into target_frame=base_footprint (Z=0 = ground)
  3. Filters: keep only points where Z ∈ [min_height, max_height]
  4. Projects surviving points onto 2D X–Y plane → LaserScan

KNOWN ISSUE — transform_tolerance
──────────────────────────────────
  If scan is empty: the node performs a TF lookup at each cloud's timestamp.
  If TF data is not available within transform_tolerance seconds, the cloud
  is silently dropped. At ~2 Hz cloud rate in Gazebo, increase to 0.5+.
  On real hardware at 30 Hz this is rarely an issue.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Scan parameters — same for both cameras (world-space Z filter) ───────────
SCAN_PARAMS = {
    "target_frame": "base_footprint",
    "min_height": 0.05,     # excludes floor (Z≈0). 5 cm clearance.
    "max_height": 10.0,     # effectively no upper limit
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.00581,  # ~0.33°
    "range_min": 0.1,       # safe — height filter handles floor
    "range_max": 6.0,
    "scan_time": 0.033,     # 30 Hz
    "transform_tolerance": 0.5,  # generous for Gazebo; fine for real robot
}


def _make_node(name, cloud_topic, scan_topic, use_sim_time):
    return Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name=name,
        output="screen",
        parameters=[{**SCAN_PARAMS, "use_sim_time": use_sim_time}],
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", scan_topic),
        ],
    )


def launch_setup(context, *args, **kwargs):
    camera = LaunchConfiguration("camera").perform(context)
    merge_scans = LaunchConfiguration("merge_scans").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = sim_time.lower() == "true"

    pkg_share = get_package_share_directory("vf_robot_slam")
    merger_config = os.path.join(pkg_share, "config", "laser_merger", "merger.yaml")
    nodes = []

    # ── Single camera: d435i ─────────────────────────────────────────────────
    if camera == "d435i":
        nodes.append(
            LogInfo(msg=[
                "\n", "=" * 70, "\n",
                "depth_to_scan [pc2scan]: D435i → /scan\n",
                "  target_frame=base_footprint, min_height=0.05 m (floor excluded)\n",
                "  No floor issue — world-space Z filter handles tilt\n",
                "=" * 70, "\n",
            ])
        )
        nodes.append(
            _make_node("pc_to_scan_d435i", "/d435i/depth/d435i_depth/points", "/scan", use_sim_time)
        )

    # ── Single camera: d455 ──────────────────────────────────────────────────
    elif camera == "d455":
        nodes.append(
            LogInfo(msg=[
                "\n", "=" * 70, "\n",
                "depth_to_scan [pc2scan]: D455 → /scan\n",
                "  target_frame=base_footprint, ~87° rear arc\n",
                "=" * 70, "\n",
            ])
        )
        nodes.append(
            _make_node("pc_to_scan_d455", "/d455/depth/d455_depth/points", "/scan", use_sim_time)
        )

    # ── Dual camera ──────────────────────────────────────────────────────────
    else:
        nodes.append(
            _make_node("pc_to_scan_d435i", "/d435i/depth/d435i_depth/points", "/scan_d435i", use_sim_time)
        )
        nodes.append(
            _make_node("pc_to_scan_d455", "/d455/depth/d455_depth/points", "/scan_d455", use_sim_time)
        )

        if merge_scans.lower() == "true":
            ira_found = False
            try:
                get_package_share_directory("ira_laser_tools")
                ira_found = True
            except Exception:
                pass

            if ira_found:
                nodes.append(
                    LogInfo(msg=[
                        "\n", "=" * 70, "\n",
                        "depth_to_scan [pc2scan]: dual — merger ACTIVE\n",
                        "  /scan_d435i + /scan_d455 → /scan (merged)\n",
                        "=" * 70, "\n",
                    ])
                )
                nodes.append(
                    Node(
                        package="ira_laser_tools",
                        executable="laserscan_multi_merger",
                        name="laserscan_multi_merger",
                        output="screen",
                        parameters=[
                            merger_config,
                            {
                                "use_sim_time": use_sim_time,
                                "laserscan_topics": "/scan_d435i /scan_d455",
                                "destination_frame": "base_footprint",
                                "scan_destination_topic": "/scan",
                                "angle_min": -3.14159,
                                "angle_max": 3.14159,
                                "range_min": 0.1,
                                "range_max": 6.0,
                            },
                        ],
                    )
                )
            else:
                # Fallback: relay D455 scan to /scan (no duplicate processing)
                nodes.append(
                    LogInfo(msg=[
                        "\n", "=" * 70, "\n",
                        "depth_to_scan [pc2scan]: dual — ira_laser_tools NOT found\n",
                        "  Fallback: /scan_d455 relayed to /scan (rear arc)\n",
                        "  /scan_d435i available for Nav2 multi-source costmap\n",
                        "  Install ira_laser_tools for merged /scan:\n",
                        "    cd ~/cogni-nav-x0/src\n",
                        "    git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2\n",
                        "    cd ~/cogni-nav-x0 && colcon build --packages-select ira_laser_tools\n",
                        "=" * 70, "\n",
                    ])
                )
                nodes.append(
                    Node(
                        package="topic_tools",
                        executable="relay",
                        name="scan_relay",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
                        arguments=["/scan_d455", "/scan"],
                    )
                )
        else:
            nodes.append(
                LogInfo(msg=[
                    "\n", "=" * 70, "\n",
                    "depth_to_scan [pc2scan]: dual — merge_scans:=false\n",
                    "  /scan_d435i + /scan_d455 published (no /scan)\n",
                    "  AMCL will not work — use Nav2 multi-source costmap\n",
                    "=" * 70, "\n",
                ])
            )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera", default_value="dual", choices=["d435i", "d455", "dual"]),
            DeclareLaunchArgument("merge_scans", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
            OpaqueFunction(function=launch_setup),
        ]
    )
