#!/usr/bin/env python3
"""
Depth → LaserScan using pc_to_scan.py — ViroFighter UVC-1
═════════════════════════════════════════════════════════════

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
    • Works correctly but may be slower than method:=dimg due to PointCloud2
      processing overhead. Use method:=dimg if frame rate is a concern.

WHY THIS BEATS depthimage_to_laserscan FOR D435i
─────────────────────────────────────────────────
  depthimage_to_laserscan collapses image rows — scan plane locked to camera
  optical axis. D435i at 60° tilt: even top image row is 31° below horizontal.
  Floor always appears as obstacles. No fix exists within that node.

  pc_to_scan.py transforms the full 3D pointcloud into base_footprint using
  TF2, then filters in WORLD Z — camera tilt is irrelevant. Floor excluded
  cleanly. range_min can be 0.1 m (no range hack needed).

IMPLEMENTATION NOTE (2026-04-06)
────────────────────────────────
  This file previously used the ros-humble-pointcloud-to-laserscan package.
  That node uses message_filters::Subscriber with a lazy subscription thread.
  In ROS 2 Humble + CycloneDDS, subscriptions created by the background
  thread after spin() starts are never processed by the executor — the node
  appears subscribed but cloudCallback never fires.

  Replaced with vf_robot_slam/pc_to_scan.py — a custom node using normal
  rclpy subscriptions with TF2 transform + numpy vectorized processing.
  No message_filters, no lazy subscription, deterministic behavior.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── Shared scan parameters — world-space Z filter, same for both cameras ─────
SCAN_PARAMS = {
    "target_frame": "base_footprint",
    "min_height": 0.02,  # 2 cm above ground — floor excluded
    "max_height": 2.0,  # 2 m — ceiling excluded
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.00581,  # ~0.33°
    "range_min": 0.1,
    "range_max": 6.0,
    "transform_tolerance": 0.1,
}


def _make_node(name, cloud_topic, scan_topic, use_sim_time):
    """Create a single pc_to_scan converter node."""
    return Node(
        package="vf_robot_slam",
        executable="pc_to_scan.py",
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
                    "  target_frame=base_footprint, min_height=0.02 m (floor excluded)\n",
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
    # No special startup order needed — pc_to_scan.py uses normal rclpy
    # subscriptions, not lazy subscription. All nodes can start together.
    # ─────────────────────────────────────────────────────────────────────────
    else:
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

        # Converter nodes — no startup order dependency
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
