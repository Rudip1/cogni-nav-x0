# =============================================================================
# vf_robot_bringup / navigation_launch.py
# =============================================================================
# Nav2 navigation stack: controller_server, planner_server, behavior_server,
# bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager.
#
# This file is modelled after nav2_bringup/navigation_launch.py but tuned
# for the ViroFighter UVC-1 robot.
#
# NOT launched directly — included by bringup_launch.py.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments ──
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Lifecycle-managed Nav2 nodes ──
    #
    # All these nodes are managed by nav2_lifecycle_manager.
    # They start in UNCONFIGURED state and lifecycle_manager
    # transitions them: configure → activate in order.

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),         # input from controller
            ('cmd_vel_smoothed', 'cmd_vel'),     # output to robot
        ],
    )

    # ── Lifecycle Manager ──
    # Manages all Nav2 nodes above.
    # Order matters: controller_server must be active before bt_navigator
    # sends goals to it.

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Nav2 params YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
    ])
