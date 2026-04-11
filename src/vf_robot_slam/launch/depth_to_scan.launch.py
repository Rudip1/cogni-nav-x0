#!/usr/bin/env python3
"""
Depth to LaserScan — Router Launch File — ViroFighter UVC-1
════════════════════════════════════════════════════════════

Single entry point for all depth-to-laserscan conversion.
Routes to the appropriate method based on the `method` argument.

Usage:
    # Gazebo simulation (fast, uses depth image — recommended for sim)
    ros2 launch vf_robot_slam depth_to_scan.launch.py method:=dimg camera:=dual

    # Real robot (accurate, uses pointcloud — recommended for production)
    ros2 launch vf_robot_slam depth_to_scan.launch.py method:=pc2scan camera:=dual

    # Real robot, single camera
    ros2 launch vf_robot_slam depth_to_scan.launch.py method:=pc2scan camera:=d455

    # Dual cameras, no merging (use Nav2 multi-source costmap instead)
    ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual merge_scans:=false

Methods:
    dimg     — depthimage_to_laserscan: processes 2D depth image, fast in Gazebo,
               D435i has 1.1 m blind zone due to 60° tilt floor intersection.
    pc2scan  — pointcloud_to_laserscan: processes 3D pointcloud with world-space
               height filter, no floor issue for D435i, but slow (~2 Hz) in Gazebo.

Outputs (all methods):
    camera:=d455                    → /scan
    camera:=d435i                   → /scan
    camera:=dual merge_scans:=true  → /scan_d435i + /scan_d455 + /scan (merged or relayed)
    camera:=dual merge_scans:=false → /scan_d435i + /scan_d455 only

Integration with vf_robot_bringup:
    The bringup package only needs to call THIS file. It does not need to know
    which underlying method is used. The `method` argument can be set by the
    user or by a top-level bringup launch file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration("method").perform(context)
    camera = LaunchConfiguration("camera").perform(context)
    merge_scans = LaunchConfiguration("merge_scans").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    pkg_share = get_package_share_directory("vf_robot_slam")

    if method == "dimg":
        include_file = os.path.join(
            pkg_share, "launch", "include", "depth_to_scan_dimg.launch.py"
        )
    else:  # pc2scan
        include_file = os.path.join(
            pkg_share, "launch", "include", "depth_to_scan_pc2scan.launch.py"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(include_file),
            launch_arguments={
                "camera": camera,
                "merge_scans": merge_scans,
                "use_sim_time": use_sim_time,
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "method",
                default_value="dimg",
                choices=["dimg", "pc2scan"],
                description=(
                    "Conversion method. "
                    "dimg: depthimage_to_laserscan (fast in Gazebo, D435i blind zone < 1.1 m). "
                    "pc2scan: pointcloud_to_laserscan (accurate, no blind zone, slow in Gazebo)."
                ),
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="dual",
                choices=["d435i", "d455", "dual"],
                description="Camera configuration.",
            ),
            DeclareLaunchArgument(
                "merge_scans",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Merge dual scans into /scan. "
                    "Uses ira_laser_tools if installed, otherwise relays D455 → /scan."
                ),
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
