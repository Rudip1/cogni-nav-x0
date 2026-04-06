# =============================================================================
# vf_robot_bringup / rviz_launch.py
# =============================================================================
# Launches RViz2 with Nav2 panels (2D Pose Estimate, Nav2 Goal).
#
# Can be launched standalone or included by bringup_launch.py.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('vf_robot_bringup')

    default_rviz = os.path.join(pkg_bringup, 'config', 'rviz', 'vf_bringup.rviz')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Full path to RViz config file',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_rviz_config,
        declare_use_sim_time,
        rviz2,
    ])
