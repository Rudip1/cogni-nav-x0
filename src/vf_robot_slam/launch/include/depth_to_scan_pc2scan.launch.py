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

STARTUP ORDER (BUG FIX — 2026-04-05)
─────────────────────────────────────
  pointcloud_to_laserscan uses LAZY SUBSCRIPTION — it will NOT subscribe
  to its pointcloud input until at least one node subscribes to its /scan
  output. If scan_merger starts AFTER the converters, the converters see
  zero subscribers and go idle. The second launch "fixes" it because the
  new scan_merger triggers the lazy check.

  FIX: In dual mode, scan_merger MUST start BEFORE the converter nodes
  so that when they check for subscribers, the merger is already listening.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Scan parameters — same for both cameras (world-space Z filter) ───────────
SCAN_PARAMS = {
    "target_frame": "base_footprint",
    "min_height": 0.02,  # 2 cm — floor excluded, small obstacles included
    "max_height": 10.0,
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.00581,
    "range_min": 0.1,
    "range_max": 6.0,
    "scan_time": 0.033,
    "transform_tolerance": 0.5,
}


def _make_node(name, cloud_topic, scan_topic, use_sim_time):
    """Create a single pointcloud_to_laserscan converter node."""
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

    nodes = []

    # ── Single camera: d435i ─────────────────────────────────────────────────
    if camera == "d435i":
        nodes.append(
            LogInfo(
                msg=[
                    "\n",
                    "=" * 70,
                    "\n",
                    "depth_to_scan [pc2scan]: D435i → /scan\n",
                    "  target_frame=base_footprint, min_height=0.05 m (floor excluded)\n",
                    "=" * 70,
                    "\n",
                ]
            )
        )
        nodes.append(
            _make_node(
                "pc_to_scan_d435i",
                "/d435i/depth/d435i_depth/points",
                "/scan",
                use_sim_time,
            )
        )

    # ── Single camera: d455 ──────────────────────────────────────────────────
    elif camera == "d455":
        nodes.append(
            LogInfo(
                msg=[
                    "\n",
                    "=" * 70,
                    "\n",
                    "depth_to_scan [pc2scan]: D455 → /scan\n",
                    "  target_frame=base_footprint, ~87° rear arc\n",
                    "=" * 70,
                    "\n",
                ]
            )
        )
        nodes.append(
            _make_node(
                "pc_to_scan_d455",
                "/d455/depth/d455_depth/points",
                "/scan",
                use_sim_time,
            )
        )

    # ── Dual camera ──────────────────────────────────────────────────────────
    #
    # CRITICAL STARTUP ORDER:
    #   1. scan_merger FIRST  — subscribes to /scan_d435i and /scan_d455
    #   2. pc_to_scan_d435i   — checks for subscribers on /scan_d435i,
    #                           finds scan_merger → activates pointcloud input
    #   3. pc_to_scan_d455    — same for /scan_d455
    #
    # If this order is reversed, the converter nodes start with zero
    # subscribers and their lazy subscription keeps them idle forever.
    # ─────────────────────────────────────────────────────────────────────────
    else:
        # ── Step 1: Start scan_merger FIRST (if merging) ─────────────────────
        if merge_scans.lower() == "true":
            nodes.append(
                LogInfo(
                    msg=[
                        "\n",
                        "=" * 70,
                        "\n",
                        "depth_to_scan [pc2scan]: dual — scan_merger ACTIVE\n",
                        "  /scan_d435i + /scan_d455 → /scan (merged)\n",
                        "=" * 70,
                        "\n",
                    ]
                )
            )
            nodes.append(
                Node(
                    package="vf_robot_slam",
                    executable="scan_merger.py",
                    name="scan_merger",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "scan_topics": "/scan_d435i /scan_d455",
                            "output_topic": "/scan",
                            "output_frame": "base_footprint",
                            "angle_min": -3.14159,
                            "angle_max": 3.14159,
                            "range_min": 0.1,
                            "range_max": 6.0,
                        }
                    ],
                )
            )

        # ── Step 2: Start converter nodes AFTER merger is subscribed ─────────
        nodes.append(
            _make_node(
                "pc_to_scan_d435i",
                "/d435i/depth/d435i_depth/points",
                "/scan_d435i",
                use_sim_time,
            )
        )
        nodes.append(
            _make_node(
                "pc_to_scan_d455",
                "/d455/depth/d455_depth/points",
                "/scan_d455",
                use_sim_time,
            )
        )

        # ── No merge: warn the user ─────────────────────────────────────────
        if merge_scans.lower() != "true":
            nodes.append(
                LogInfo(
                    msg=[
                        "\n",
                        "=" * 70,
                        "\n",
                        "depth_to_scan [pc2scan]: dual — merge_scans:=false\n",
                        "  /scan_d435i + /scan_d455 published (no /scan)\n",
                        "  AMCL will not work — use Nav2 multi-source costmap\n",
                        "=" * 70,
                        "\n",
                    ]
                )
            )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera", default_value="dual", choices=["d435i", "d455", "dual"]
            ),
            DeclareLaunchArgument(
                "merge_scans", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
