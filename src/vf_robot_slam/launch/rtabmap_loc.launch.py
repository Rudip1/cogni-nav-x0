#!/usr/bin/env python3
"""
RTAB-Map Localization Launch File for ViroFighter UVC-1

Usage:
    ros2 launch vf_robot_slam rtabmap_loc.launch.py camera:=dual map_name:=my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_rtabmap_loc_params(database_path):
    """RTAB-Map parameters for Localization mode."""
    return {
        # Frame configuration
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        
        # General
        'queue_size': 10,
        'approx_sync': True,
        
        # Database
        'database_path': database_path,
        
        # Localization mode (don't add new nodes)
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',
        
        # Optimizer
        'Optimizer/Strategy': '1',
        'Optimizer/Iterations': '20',
        'Optimizer/Slam2D': 'true',
        
        # Registration
        'Reg/Strategy': '0',
        'Reg/Force3DoF': 'true',
        
        # Visual features
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',
        'Vis/MaxFeatures': '500',
        'Vis/FeatureType': '8',
        
        # Loop closure (more sensitive for localization)
        'Rtabmap/DetectionRate': '2.0',
        'Rtabmap/LoopThr': '0.09',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'RGBD/OptimizeMaxError': '3.0',
        
        # Don't update map
        'RGBD/LinearUpdate': '0.0',
        'RGBD/AngularUpdate': '0.0',
        
        # Memory
        'Mem/ImageKept': 'false',
        'Mem/STMSize': '10',
        
        # Optimize from last known position
        'RGBD/OptimizeFromGraphEnd': 'true',
    }


def launch_setup(context, *args, **kwargs):
    camera = LaunchConfiguration('camera').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)
    maps_dir = LaunchConfiguration('maps_dir').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    
    maps_dir = os.path.expanduser(maps_dir)
    map_folder = os.path.join(maps_dir, map_name)
    database_path = os.path.join(map_folder, f"{map_name}.db")
    
    pkg_share = get_package_share_directory('vf_robot_slam')
    
    nodes = []
    
    # Check if database exists
    if not os.path.exists(database_path):
        nodes.append(LogInfo(msg=[
            '\n',
            '=' * 70, '\n',
            'ERROR: Map database not found!\n',
            '=' * 70, '\n',
            f'Expected: {database_path}\n',
            '\n',
            'To create a map first, run:\n',
            f'  ros2 launch vf_robot_slam rtabmap_slam.launch.py map_name:={map_name}\n',
            '=' * 70, '\n',
        ]))
        return nodes
    
    nodes.append(LogInfo(msg=[
        '\n',
        '=' * 70, '\n',
        'RTAB-Map Localization Mode\n',
        '=' * 70, '\n',
        f'Camera:        {camera}\n',
        f'Map name:      {map_name}\n',
        f'Database:      {database_path}\n',
        '=' * 70, '\n',
    ]))
    
    rtabmap_params = get_rtabmap_loc_params(database_path)
    
    if camera == 'dual':
        # RGBD Sync for D435i
        nodes.append(Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_d435i',
            output='screen',
            parameters=[{'approx_sync': True, 'queue_size': 10}],
            remappings=[
                ('rgb/image', '/d435i/rgb/d435i_rgb/image_raw'),
                ('rgb/camera_info', '/d435i/rgb/d435i_rgb/camera_info'),
                ('depth/image', '/d435i/depth/d435i_depth/depth/image_raw'),
                ('rgbd_image', '/rgbd_image/d435i'),
            ],
        ))
        
        # RGBD Sync for D455
        nodes.append(Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_d455',
            output='screen',
            parameters=[{'approx_sync': True, 'queue_size': 10}],
            remappings=[
                ('rgb/image', '/d455/rgb/d455_rgb/image_raw'),
                ('rgb/camera_info', '/d455/rgb/d455_rgb/camera_info'),
                ('depth/image', '/d455/depth/d455_depth/depth/image_raw'),
                ('rgbd_image', '/rgbd_image/d455'),
            ],
        ))
        
        # RTAB-Map for dual cameras
        dual_params = rtabmap_params.copy()
        dual_params.update({
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': True,
            'rgbd_cameras': 2,
        })
        
        nodes.append(Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[dual_params],
            remappings=[
                ('rgbd_image0', '/rgbd_image/d435i'),
                ('rgbd_image1', '/rgbd_image/d455'),
                ('odom', '/odom'),
                ('map', '/map'),
            ],
        ))
        
    else:
        # Single camera mode
        if camera == 'd435i':
            rgb_topic = '/d435i/rgb/d435i_rgb/image_raw'
            rgb_info_topic = '/d435i/rgb/d435i_rgb/camera_info'
            depth_topic = '/d435i/depth/d435i_depth/depth/image_raw'
        else:  # d455
            rgb_topic = '/d455/rgb/d455_rgb/image_raw'
            rgb_info_topic = '/d455/rgb/d455_rgb/camera_info'
            depth_topic = '/d455/depth/d455_depth/depth/image_raw'
        
        single_params = rtabmap_params.copy()
        single_params.update({
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
        })
        
        nodes.append(Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[single_params],
            remappings=[
                ('rgb/image', rgb_topic),
                ('rgb/camera_info', rgb_info_topic),
                ('depth/image', depth_topic),
                ('odom', '/odom'),
                ('map', '/map'),
            ],
        ))
    
    # RViz
    if rviz.lower() == 'true':
        rviz_config = os.path.join(pkg_share, 'rviz', 'rtabmap_loc.rviz')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ))
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='dual',
            choices=['d435i', 'd455', 'dual'],
            description='Camera configuration'),
        DeclareLaunchArgument('map_name', default_value='',
            description='Name of the map to load (REQUIRED)'),
        DeclareLaunchArgument('maps_dir', default_value='~/cogni-nav-x0/maps',
            description='Base directory for maps'),
        DeclareLaunchArgument('rviz', default_value='true',
            choices=['true', 'false'],
            description='Launch RViz'),
        OpaqueFunction(function=launch_setup),
    ])
