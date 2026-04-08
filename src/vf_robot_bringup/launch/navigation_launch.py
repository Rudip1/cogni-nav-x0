#!/usr/bin/env python3

# =============================================================================
# vf_robot_bringup / launch / navigation_launch.py
# =============================================================================
# Nav2 navigation stack — the controller/planner/BT/behaviors/smoother/
# velocity_smoother/collision_monitor/waypoint_follower pipeline, all
# managed by nav2_lifecycle_manager.
#
# This launch file is INCLUDED by bringup_launch.py — not meant to run
# standalone. The params_file argument comes from bringup_launch.py's
# compose_params step and points at /tmp/nav2_<label>_xxx.yaml.
#
# TOPIC WIRING (important for understanding the cmd_vel chain):
#
#   controller_server   publishes →  cmd_vel_nav
#                                       ↓
#   velocity_smoother   subscribes →  cmd_vel_nav
#                       publishes  →  cmd_vel_smoothed
#                                       ↓
#   collision_monitor   subscribes →  cmd_vel_smoothed
#                       publishes  →  cmd_vel
#                                       ↓
#   Gazebo diff drive / robot driver →  cmd_vel
#
# The remapping of controller_server's "cmd_vel" output to "cmd_vel_nav"
# is necessary because both velocity_smoother and collision_monitor want
# to consume from a Nav2-side topic without the robot's driver seeing it.
# collision_monitor owns the FINAL publish to "cmd_vel" — everything
# between the controller and the robot flows through this chain.
#
# The collision_monitor topic wiring (cmd_vel_smoothed → cmd_vel) is set
# INSIDE the composed yaml (nav2_base.yaml), not via remaps here. Only
# controller_server and velocity_smoother need remaps at the node level.
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file  = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── controller_server ───────────────────────────────────────────────
    # Owns the local controller plugin (FollowPath) — whichever one the
    # composed params file specifies. Publishes velocity commands to
    # cmd_vel_nav (remapped from the default cmd_vel) so they flow
    # through the smoother/collision_monitor chain.
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # ── planner_server ──────────────────────────────────────────────────
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── smoother_server ─────────────────────────────────────────────────
    # Post-planner path smoother (runs on the GLOBAL path, not cmd_vel).
    # Don't confuse this with velocity_smoother below — different node,
    # different purpose.
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── behavior_server ─────────────────────────────────────────────────
    # Recovery behaviors: spin, backup, drive_on_heading, wait, assisted_teleop.
    # NOTE: behavior_server publishes DIRECTLY to /cmd_vel, bypassing the
    # smoother/collision_monitor chain. This is intentional in Nav2 —
    # recoveries need to be able to move the robot even if the main
    # controller cmd_vel chain is stuck. Just be aware of it.
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── bt_navigator ────────────────────────────────────────────────────
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── waypoint_follower ───────────────────────────────────────────────
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── velocity_smoother ───────────────────────────────────────────────
    # Reads cmd_vel_nav (from controller), publishes cmd_vel_smoothed
    # (to collision_monitor). The remaps below rename the node's default
    # topics to match this chain.
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel",          "cmd_vel_nav"),        # input
            ("cmd_vel_smoothed", "cmd_vel_smoothed"),   # output (explicit)
        ],
    )

    # ── collision_monitor ───────────────────────────────────────────────
    # Safety gate between velocity_smoother and the robot. Reads
    # cmd_vel_smoothed, watches the local costmap, and passes through
    # to cmd_vel — but will clamp or stop the command if a collision
    # is imminent. The cmd_vel_in_topic / cmd_vel_out_topic wiring is
    # set in nav2_base.yaml under collision_monitor.ros__parameters.
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── lifecycle_manager ───────────────────────────────────────────────
    # Activates every Nav2 node in the order listed below. Order matters
    # for dependency reasons:
    #   1. controller_server         (owns local costmap)
    #   2. smoother_server           (post-plan smoothing)
    #   3. planner_server            (owns global costmap)
    #   4. behavior_server           (uses local costmap topic)
    #   5. bt_navigator              (orchestrates everyone above)
    #   6. waypoint_follower         (uses bt_navigator action)
    #   7. velocity_smoother         (consumes controller output)
    #   8. collision_monitor         (final gate before robot)
    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
                "collision_monitor",
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", description="Composed Nav2 params YAML"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_navigation,
    ])
