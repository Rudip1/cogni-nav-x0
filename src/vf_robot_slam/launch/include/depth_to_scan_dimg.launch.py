#!/usr/bin/env python3
"""
Depth → LaserScan using depthimage_to_laserscan — ViroFighter UVC-1
════════════════════════════════════════════════════════════════════════

This file is included by depth_to_scan.launch.py (method:=dimg).
Do NOT launch directly — use:
    ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=dual

WHEN TO USE
───────────
  Simulation (Gazebo):
    • Processes cheap depth IMAGE (not heavy PointCloud2)
    • Runs at full 15–30 Hz even in Gazebo — no CPU bottleneck
    • This is the recommended method for simulation

  Real robot — D455 only:
    • D455 is horizontal at 0.429 m — scan plane at correct obstacle height
    • Works perfectly

  Real robot — D435i:
    • D435i at 1.773 m, 60° tilt → scan plane hits floor at ~1.02 m
    • range_min=1.1 m clips floor returns but also clips obstacles < 1.1 m
    • For close-obstacle detection use method:=pc2scan instead

D435i LIMITATION
────────────────
  Camera at 1.773 m, 60° tilt downward.
  Optical axis hits floor at: 1.773 / tan(60°) = 1.024 m.
  Even the topmost image row is 31° below horizontal
  → floor intersection at 2.95 m — still ground.
  There is NO image row that sees a useful obstacle-height plane.
  Workaround: range_min = 1.1 m clips floor at ≤1.024 m.
  Tradeoff: obstacles closer than 1.1 m are invisible to this scan.

output_frame notes
──────────────────
  D455: camera_d455_link — faces rear, so angle=0 draws backward ✓
        DO NOT use base_footprint — depthimage_to_laserscan does NOT
        rotate angle values, only stamps header.frame_id.
  D435i: base_footprint — faces forward, angle=0 = +X_base = forward ✓
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── D455 parameters — horizontal, 0.429 m height ────────────────────────────
D455_PARAMS = {
    "range_min": 0.6,
    "range_max": 6.0,
    "scan_height": 1,
    "scan_time": 0.033,
    "inf_is_valid": False,
    "output_frame": "camera_d455_link",
}

# ── D435i parameters — 60° tilt, 1.773 m height ─────────────────────────────
D435I_PARAMS = {
    "range_min": 1.1,   # clips floor at ≤1.024 m; blind zone < 1.1 m
    "range_max": 6.0,
    "scan_height": 1,
    "scan_time": 0.033,
    "inf_is_valid": False,
    "output_frame": "base_footprint",
}


def _make_d455_node(name, scan_topic, use_sim_time):
    return Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name=name,
        output="screen",
        parameters=[{**D455_PARAMS, "use_sim_time": use_sim_time}],
        remappings=[
            ("depth", "/d455/depth/d455_depth/depth/image_raw"),
            ("depth_camera_info", "/d455/depth/d455_depth/depth/camera_info"),
            ("scan", scan_topic),
        ],
    )


def _make_d435i_node(name, scan_topic, use_sim_time):
    return Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name=name,
        output="screen",
        parameters=[{**D435I_PARAMS, "use_sim_time": use_sim_time}],
        remappings=[
            ("depth", "/d435i/depth/d435i_depth/depth/image_raw"),
            ("depth_camera_info", "/d435i/depth/d435i_depth/depth/camera_info"),
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
                "depth_to_scan [dimg]: D435i → /scan\n",
                "  range_min=1.1 m (floor clipped), blind zone < 1.1 m\n",
                "=" * 70, "\n",
            ])
        )
        nodes.append(_make_d435i_node("depth_to_scan_d435i", "/scan", use_sim_time))

    # ── Single camera: d455 ──────────────────────────────────────────────────
    elif camera == "d455":
        nodes.append(
            LogInfo(msg=[
                "\n", "=" * 70, "\n",
                "depth_to_scan [dimg]: D455 → /scan\n",
                "  output_frame=camera_d455_link, range_min=0.6 m\n",
                "=" * 70, "\n",
            ])
        )
        nodes.append(_make_d455_node("depth_to_scan_d455", "/scan", use_sim_time))

    # ── Dual camera ──────────────────────────────────────────────────────────
    else:
        nodes.append(_make_d435i_node("depth_to_scan_d435i", "/scan_d435i", use_sim_time))
        nodes.append(_make_d455_node("depth_to_scan_d455", "/scan_d455", use_sim_time))

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
                        "depth_to_scan [dimg]: dual — merger ACTIVE\n",
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
                                "range_min": 0.6,
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
                        "depth_to_scan [dimg]: dual — ira_laser_tools NOT found\n",
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
                    "depth_to_scan [dimg]: dual — merge_scans:=false\n",
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
