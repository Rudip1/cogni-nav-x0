# =============================================================================
# vf_robot_bringup / slam_launch.py
# =============================================================================
# Mode 4: SLAM Toolbox — builds a map from /scan using laser-based SLAM.
#
# Starts:
#   - slam_toolbox:  online_async_launch — subscribes to /scan,
#                    publishes /map + map→odom TF
#   - lifecycle_manager: manages slam_toolbox
#
# Requires:
#   - /scan topic (from depth_to_scan.launch.py)
#   - /odom and odom→base_footprint TF (from Gazebo or robot driver)
#
# NOT launched directly — included by bringup_launch.py.
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── SLAM Toolbox (online async) ──
    # Builds a 2D occupancy grid from /scan.
    # Publishes /map and map→odom TF.
    # "online_async" mode: processes scans asynchronously for better performance.

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    # ── Lifecycle Manager ──
    # SLAM Toolbox in Humble supports lifecycle management.

    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'slam_toolbox',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Nav2 params YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        slam_toolbox_node,
        lifecycle_manager_slam,
    ])
