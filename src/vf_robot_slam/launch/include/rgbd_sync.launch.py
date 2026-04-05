#!/usr/bin/env python3
"""
Reusable RGBD Sync Launch — ViroFighter UVC-1
══════════════════════════════════════════════

Spawns rtabmap_sync/rgbd_sync nodes for the selected camera configuration.
This file is NOT launched directly — it is included by rtabmap_slam.launch.py
and rtabmap_loc.launch.py so the sync node definitions live in ONE place.

Outputs:
    camera:=d435i  →  /rgbd_image/d435i
    camera:=d455   →  /rgbd_image/d455
    camera:=dual   →  /rgbd_image/d435i + /rgbd_image/d455

Critical notes (hard-won lessons):
    • depth/camera_info MUST be remapped — without it rgbd_sync produces
      RGBDImage messages with empty frame_ids, causing silent failures.
    • approx_sync_max_interval: 0.05 — setting 0.0 defeats approx_sync
      on some rtabmap_sync versions, dropping frames to ~6 Hz.
    • queue_size: 30 — prevents drops when RTAB-Map processing is slow.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_sync_params(use_sim_time):
    """Shared parameters for all rgbd_sync nodes."""
    return {
        "use_sim_time": use_sim_time,
        "approx_sync": True,
        "approx_sync_max_interval": 0.05,  # 50 ms tolerance (0.0 defeats sync)
        "queue_size": 30,
    }


def _make_sync_node(name, rgb, rgb_info, depth, depth_info, rgbd_out, use_sim_time):
    """Create a single rgbd_sync node with full remappings."""
    return Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name=name,
        output="screen",
        parameters=[_get_sync_params(use_sim_time)],
        remappings=[
            ("rgb/image", rgb),
            ("rgb/camera_info", rgb_info),
            ("depth/image", depth),
            ("depth/camera_info", depth_info),
            ("rgbd_image", rgbd_out),
        ],
    )


def launch_setup(context, *args, **kwargs):
    camera = LaunchConfiguration("camera").perform(context)
    sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = sim_time.lower() == "true"

    nodes = []

    if camera in ("d435i", "dual"):
        nodes.append(
            _make_sync_node(
                name="rgbd_sync_d435i",
                rgb="/d435i/rgb/d435i_rgb/image_raw",
                rgb_info="/d435i/rgb/d435i_rgb/camera_info",
                depth="/d435i/depth/d435i_depth/depth/image_raw",
                depth_info="/d435i/depth/d435i_depth/depth/camera_info",
                rgbd_out="/rgbd_image/d435i",
                use_sim_time=use_sim_time,
            )
        )

    if camera in ("d455", "dual"):
        nodes.append(
            _make_sync_node(
                name="rgbd_sync_d455",
                rgb="/d455/rgb/d455_rgb/image_raw",
                rgb_info="/d455/rgb/d455_rgb/camera_info",
                depth="/d455/depth/d455_depth/depth/image_raw",
                depth_info="/d455/depth/d455_depth/depth/camera_info",
                rgbd_out="/rgbd_image/d455",
                use_sim_time=use_sim_time,
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                default_value="dual",
                choices=["d435i", "d455", "dual"],
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
