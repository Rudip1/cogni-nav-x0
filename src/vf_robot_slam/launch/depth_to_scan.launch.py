#!/usr/bin/env python3
"""
Depth to LaserScan Launch File for ViroFighter UVC-1

Usage:
    ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=d435i
    ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=d455
    ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual
    ros2 launch vf_robot_slam depth_to_scan.launch.py camera:=dual merge_scans:=false

Arguments:
    camera: Camera configuration (d435i, d455, or dual). Default: dual
    merge_scans: Merge dual scans into /scan (requires ira_laser_tools). Default: true
    
Outputs:
    camera:=d435i              → /scan (from D435i depth)
    camera:=d455               → /scan (from D455 depth)
    camera:=dual merge:=true   → /scan_d435i, /scan_d455, /scan (merged)
    camera:=dual merge:=false  → /scan_d435i, /scan_d455 (no merge, use Nav2 multi-source)

Notes:
    - D435i is tilted 60° down at 1.773m - scan may detect floor, not obstacles
    - D455 is horizontal at 0.429m - better for obstacle detection
    - In dual mode with merge_scans:=true, ira_laser_tools merges scans for AMCL
    - In dual mode with merge_scans:=false, configure Nav2 costmap with multiple sources
    
    ira_laser_tools installation (required for merge_scans:=true):
        cd ~/your_ws/src
        git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2
        colcon build --packages-select ira_laser_tools
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function that evaluates launch configurations."""
    
    camera = LaunchConfiguration('camera').perform(context)
    merge_scans = LaunchConfiguration('merge_scans').perform(context)
    
    pkg_share = get_package_share_directory('vf_robot_slam')
    
    # Load config files
    common_config = os.path.join(pkg_share, 'config', 'depth_to_scan', 'common.yaml')
    d435i_config = os.path.join(pkg_share, 'config', 'depth_to_scan', 'd435i.yaml')
    d455_config = os.path.join(pkg_share, 'config', 'depth_to_scan', 'd455.yaml')
    merger_config = os.path.join(pkg_share, 'config', 'laser_merger', 'merger.yaml')
    
    nodes = []
    
    if camera == 'd435i':
        # Single D435i camera → /scan
        depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[
                common_config,
                d435i_config,
                {
                    'output_frame_id': 'camera_d435i_depth_optical_frame',
                    'range_min': 0.6,
                    'range_max': 6.0,
                },
            ],
            remappings=[
                ('depth', '/d435i/depth/d435i_depth/depth/image_raw'),
                ('depth_camera_info', '/d435i/depth/d435i_depth/depth/camera_info'),
                ('scan', '/scan'),
            ],
        )
        nodes.append(depth_to_scan)
        
    elif camera == 'd455':
        # Single D455 camera → /scan
        depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[
                common_config,
                d455_config,
                {
                    'output_frame_id': 'camera_d455_depth_optical_frame',
                    'range_min': 0.6,
                    'range_max': 6.0,
                },
            ],
            remappings=[
                ('depth', '/d455/depth/d455_depth/depth/image_raw'),
                ('depth_camera_info', '/d455/depth/d455_depth/depth/camera_info'),
                ('scan', '/scan'),
            ],
        )
        nodes.append(depth_to_scan)
        
    else:  # dual
        # D435i → /scan_d435i
        depth_to_scan_d435i = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan_d435i',
            output='screen',
            parameters=[
                common_config,
                d435i_config,
                {
                    'output_frame_id': 'camera_d435i_depth_optical_frame',
                    'range_min': 0.6,
                    'range_max': 6.0,
                },
            ],
            remappings=[
                ('depth', '/d435i/depth/d435i_depth/depth/image_raw'),
                ('depth_camera_info', '/d435i/depth/d435i_depth/depth/camera_info'),
                ('scan', '/scan_d435i'),
            ],
        )
        nodes.append(depth_to_scan_d435i)
        
        # D455 → /scan_d455
        depth_to_scan_d455 = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan_d455',
            output='screen',
            parameters=[
                common_config,
                d455_config,
                {
                    'output_frame_id': 'camera_d455_depth_optical_frame',
                    'range_min': 0.6,
                    'range_max': 6.0,
                },
            ],
            remappings=[
                ('depth', '/d455/depth/d455_depth/depth/image_raw'),
                ('depth_camera_info', '/d455/depth/d455_depth/depth/camera_info'),
                ('scan', '/scan_d455'),
            ],
        )
        nodes.append(depth_to_scan_d455)
        
        # Optionally merge scans (requires ira_laser_tools)
        if merge_scans.lower() == 'true':
            # Check if ira_laser_tools is available
            try:
                from ament_index_python.packages import get_package_share_directory
                get_package_share_directory('ira_laser_tools')
                
                # Merge /scan_d435i + /scan_d455 → /scan
                laser_merger = Node(
                    package='ira_laser_tools',
                    executable='laserscan_multi_merger',
                    name='laserscan_multi_merger',
                    output='screen',
                    parameters=[
                        merger_config,
                        {
                            'laserscan_topics': '/scan_d435i /scan_d455',
                            'destination_frame': 'base_link',
                            'scan_destination_topic': '/scan',
                            'angle_min': -3.14159,
                            'angle_max': 3.14159,
                            'range_min': 0.6,
                            'range_max': 6.0,
                        },
                    ],
                )
                nodes.append(laser_merger)
                
            except Exception:
                # ira_laser_tools not installed - print warning
                nodes.append(LogInfo(msg=[
                    '\n',
                    '=' * 70, '\n',
                    'WARNING: ira_laser_tools not found!\n',
                    '=' * 70, '\n',
                    'Scan merging disabled. Only /scan_d435i and /scan_d455 available.\n',
                    'No /scan topic will be published.\n',
                    '\n',
                    'To install ira_laser_tools:\n',
                    '  cd ~/your_ws/src\n',
                    '  git clone https://github.com/iralabdisco/ira_laser_tools.git -b ros2\n',
                    '  colcon build --packages-select ira_laser_tools\n',
                    '\n',
                    'Alternative: Use merge_scans:=false and configure Nav2 costmap\n',
                    'with multiple observation sources (see README.md)\n',
                    '=' * 70, '\n',
                ]))
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'camera',
            default_value='dual',
            choices=['d435i', 'd455', 'dual'],
            description='Camera configuration: d435i (front), d455 (rear), or dual (both)'
        ),
        
        DeclareLaunchArgument(
            'merge_scans',
            default_value='true',
            choices=['true', 'false'],
            description='Merge dual camera scans into /scan (requires ira_laser_tools)'
        ),
        
        # Use OpaqueFunction to evaluate conditions at runtime
        OpaqueFunction(function=launch_setup),
    ])
