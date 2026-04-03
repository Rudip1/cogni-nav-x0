#!/usr/bin/env python3
"""
RTAB-Map SLAM Launch File for ViroFighter UVC-1

Usage:
    # New map (deletes existing)
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office

    # Continue existing map
    ros2 launch vf_robot_slam rtabmap_slam.launch.py camera:=dual map_name:=my_office new_map:=false
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_rtabmap_params(database_path, delete_db):
    """Common RTAB-Map parameters for SLAM mode."""
    return {
        # Frame configuration
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "publish_tf": True,
        # General
        "queue_size": 10,
        "approx_sync": True,
        # Database
        "database_path": database_path,
        "delete_db_on_start": delete_db,  # NEW: delete or continue
        # SLAM mode
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        # FIX FOR MULTI-CAMERA WITHOUT OPENGV
        # Use 3D-3D registration instead of 2D-3D PnP
        "Reg/Strategy": "1",  # 1=ICP (works without OpenGV)
        "Vis/EstimationType": "0",  # 0=3D-3D (not PnP which needs OpenGV)
        "Reg/Force3DoF": "true",  # 2D SLAM for ground robot
        # Optimizer
        "Optimizer/Strategy": "1",
        "Optimizer/Iterations": "20",
        "Optimizer/Slam2D": "true",
        # Visual features
        "Vis/MinInliers": "15",
        "Vis/InlierDistance": "0.1",
        "Vis/MaxFeatures": "500",
        "Vis/FeatureType": "8",
        # Loop closure
        "Rtabmap/DetectionRate": "1.0",
        "Rtabmap/TimeThr": "0.0",
        "Rtabmap/LoopThr": "0.11",
        "RGBD/LoopClosureReextractFeatures": "true",
        "RGBD/OptimizeMaxError": "3.0",
        # Mapping thresholds
        "RGBD/LinearUpdate": "0.1",
        "RGBD/AngularUpdate": "0.1",
        "RGBD/CreateOccupancyGrid": "true",
        # Memory
        "Mem/ImageKept": "true",
        "Mem/STMSize": "30",
        # Grid map
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

    maps_dir = os.path.expanduser(maps_dir)
    map_folder = os.path.join(maps_dir, map_name)
    database_path = os.path.join(map_folder, f"{map_name}.db")

    Path(map_folder).mkdir(parents=True, exist_ok=True)

    pkg_share = get_package_share_directory("vf_robot_slam")

    # Determine if we should delete existing database
    delete_db = new_map.lower() == "true"
    mode_str = "NEW MAP (deleting existing)" if delete_db else "CONTINUING existing map"

    nodes = []

    nodes.append(
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
                "\n",
                "To save 2D map for AMCL (while running):\n",
                f"  ros2 run nav2_map_server map_saver_cli -f {map_folder}/{map_name}\n",
                "=" * 70,
                "\n",
            ]
        )
    )

    rtabmap_params = get_rtabmap_params(database_path, delete_db)

    if camera == "dual":
        # RGBD Sync for D435i
        nodes.append(
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync_d435i",
                output="screen",
                parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.0}],
                remappings=[
                    ("rgb/image", "/d435i/rgb/d435i_rgb/image_raw"),
                    ("rgb/camera_info", "/d435i/rgb/d435i_rgb/camera_info"),
                    ("depth/image", "/d435i/depth/d435i_depth/depth/image_raw"),
                    ("rgbd_image", "/rgbd_image/d435i"),
                ],
            )
        )

        # RGBD Sync for D455
        nodes.append(
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                name="rgbd_sync_d455",
                output="screen",
                parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.0}],
                remappings=[
                    ("rgb/image", "/d455/rgb/d455_rgb/image_raw"),
                    ("rgb/camera_info", "/d455/rgb/d455_rgb/camera_info"),
                    ("depth/image", "/d455/depth/d455_depth/depth/image_raw"),
                    ("rgbd_image", "/rgbd_image/d455"),
                ],
            )
        )

        # RTAB-Map for dual cameras
        dual_params = rtabmap_params.copy()
        dual_params.update(
            {
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_rgbd": True,
                "rgbd_cameras": 2,
            }
        )

        nodes.append(
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[dual_params],
                remappings=[
                    ("rgbd_image0", "/rgbd_image/d435i"),
                    ("rgbd_image1", "/rgbd_image/d455"),
                    ("odom", "/odom"),
                    ("map", "/map"),
                ],
            )
        )

    else:
        # Single camera mode
        if camera == "d435i":
            rgb_topic = "/d435i/rgb/d435i_rgb/image_raw"
            rgb_info_topic = "/d435i/rgb/d435i_rgb/camera_info"
            depth_topic = "/d435i/depth/d435i_depth/depth/image_raw"
        else:  # d455
            rgb_topic = "/d455/rgb/d455_rgb/image_raw"
            rgb_info_topic = "/d455/rgb/d455_rgb/camera_info"
            depth_topic = "/d455/depth/d455_depth/depth/image_raw"

        single_params = rtabmap_params.copy()
        single_params.update(
            {
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_rgbd": False,
            }
        )

        nodes.append(
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[single_params],
                remappings=[
                    ("rgb/image", rgb_topic),
                    ("rgb/camera_info", rgb_info_topic),
                    ("depth/image", depth_topic),
                    ("odom", "/odom"),
                    ("map", "/map"),
                ],
            )
        )

    # RViz
    if rviz.lower() == "true":
        rviz_config = os.path.join(pkg_share, "rviz", "rtabmap_slam.rviz")
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
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
                description="Camera configuration",
            ),
            DeclareLaunchArgument(
                "map_name", default_value="default_map", description="Name for the map"
            ),
            DeclareLaunchArgument(
                "maps_dir",
                default_value="~/cogni-nav-x0/maps",
                description="Base directory for maps",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                choices=["true", "false"],
                description="Launch RViz",
            ),
            DeclareLaunchArgument(
                "new_map",
                default_value="true",
                choices=["true", "false"],
                description="Start fresh map (true) or continue existing (false)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
