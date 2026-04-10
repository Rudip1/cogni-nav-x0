#!/usr/bin/env python3

# =============================================================================
# vf_robot_bringup / launch / bringup_launch.py
# =============================================================================
# THE single entry point for ViroFighter (and any other) robot navigation.
#
# Three orthogonal axes, one launch command:
#
#   robot:=        which robot profile to use
#                  → virofighter | turtlebot3_waffle
#
#   controller:=   which controller plugin to load
#                  → vf_fixed | vf_inference | vf_collect | vf_passive
#                    | mppi | dwb
#
#   localization:= which SLAM/localization mode to run
#                  → rtabmap_slam | rtabmap_loc | amcl | slam_toolbox
#
# Any combination is valid. Examples:
#
#   # ViroFighter + your meta-critic plugin + RTAB-Map localization (the thesis demo)
#   ros2 launch vf_robot_bringup bringup_launch.py \
#       robot:=virofighter controller:=vf_inference localization:=rtabmap_loc \
#       map_name:=hospital
#
#   # ViroFighter + stock Nav2 MPPI + RTAB-Map (primary baseline)
#   ros2 launch vf_robot_bringup bringup_launch.py \
#       robot:=virofighter controller:=mppi localization:=rtabmap_loc \
#       map_name:=hospital
#
#   # ViroFighter + your plugin in fixed mode + AMCL (ablation baseline)
#   ros2 launch vf_robot_bringup bringup_launch.py \
#       robot:=virofighter controller:=vf_fixed localization:=amcl \
#       map:=$HOME/cogni-nav-x0/maps/hospital/hospital.yaml
#
#   # TurtleBot3 + your plugin in inference mode + SLAM Toolbox (portability proof)
#   ros2 launch vf_robot_bringup bringup_launch.py \
#       robot:=turtlebot3_waffle controller:=vf_inference localization:=slam_toolbox
#
#   # Real robot — any mode:
#   ros2 launch vf_robot_bringup bringup_launch.py \
#       robot:=virofighter controller:=vf_inference localization:=rtabmap_loc \
#       map_name:=hospital use_sim_time:=false
#
# Prerequisites:
#   Gazebo (sim) or real robot driver must already be running.
#   e.g.  ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py
#
# How it works internally:
#   1. compose_params() reads nav2_base.yaml + the chosen controller fragment
#      + (optionally) the chosen localization fragment, then applies the
#      robot profile's rewrites. The result is written to /tmp/nav2_*.yaml.
#   2. depth_to_scan is started unconditionally (all SLAM modes need /scan).
#   3. Exactly ONE of the four SLAM/localization branches is included
#      based on `localization:=`.
#   4. navigation_launch.py is started with the composed params file.
#   5. RViz is started (if rviz:=true).
#   6. If autostart_sidecar:=true AND controller is one of vf_inference/
#      vf_collect/vf_passive, the corresponding Python sidecar from
#      vf_robot_controller is also included. Default is false; users
#      typically run the sidecar in a separate (conda dl) terminal.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ============================================================================
# Argument value lists — keep in sync with config/ contents
# ============================================================================

VALID_ROBOTS = [
    "virofighter",
    "turtlebot3_waffle",
]

VALID_CONTROLLERS = [
    "vf_fixed",
    "vf_inference",
    "vf_collect",
    "vf_passive",
    "mppi",
    "dwb",
]

VALID_LOCALIZATIONS = [
    "rtabmap_slam",
    "rtabmap_loc",
    "amcl",
    "slam_toolbox",
]

# Maps a controller name to its parameter-rewrite family.
# (vf_* fragments share the same family because they use the same plugin.)
CONTROLLER_FAMILY = {
    "vf_fixed":     "vf",
    "vf_inference": "vf",
    "vf_collect":   "vf",
    "vf_passive":   "vf",
    "mppi":         "mppi",
    "dwb":          "dwb",
}

# Controllers that require a Python sidecar from vf_robot_controller.
# These need to be told the user about (or auto-launched if opted in).
SIDECAR_CONTROLLERS = {
    "vf_inference": "meta_critic_inference_launch.py",
    "vf_collect":   "meta_critic_collect_launch.py",
    "vf_passive":   "imitation_inference_launch.py",
}


# ============================================================================
# OpaqueFunction: compose params at launch time
# ============================================================================

def _compose_action(context, *args, **kwargs):
    """
    Called by ROS launch *after* arguments are resolved but *before* nodes
    spawn. Reads the resolved robot/controller/localization values, calls
    the composer, and stashes the resulting /tmp/ path in a launch
    configuration named `composed_params_file` so the rest of the launch
    description can pass it to navigation/localization/slam launches.

    Also performs sidecar reminder logging here (rather than as a separate
    LogInfo) because we need the resolved controller value to know whether
    a sidecar is required.
    """
    # Resolve all the arguments we need
    robot         = LaunchConfiguration("robot").perform(context)
    controller    = LaunchConfiguration("controller").perform(context)
    localization  = LaunchConfiguration("localization").perform(context)
    autostart_str = LaunchConfiguration("autostart_sidecar").perform(context)

    # Validate (defense in depth — choices=[] should already catch this,
    # but a clearer error here helps if launch is invoked programmatically)
    if robot not in VALID_ROBOTS:
        raise RuntimeError(
            f"[vf_bringup] Invalid robot '{robot}'. "
            f"Valid: {VALID_ROBOTS}"
        )
    if controller not in VALID_CONTROLLERS:
        raise RuntimeError(
            f"[vf_bringup] Invalid controller '{controller}'. "
            f"Valid: {VALID_CONTROLLERS}"
        )
    if localization not in VALID_LOCALIZATIONS:
        raise RuntimeError(
            f"[vf_bringup] Invalid localization '{localization}'. "
            f"Valid: {VALID_LOCALIZATIONS}"
        )

    # Lazy import — keeps the launch file importable even if compose_params
    # has a transient bug, until this function actually runs
    from vf_robot_bringup.launch_utils.compose_params import (
        compose,
        load_robot_profile,
    )

    pkg_bringup = get_package_share_directory("vf_robot_bringup")
    config_root = os.path.join(pkg_bringup, "config")

    base_path       = os.path.join(config_root, "nav2", "nav2_base.yaml")
    controller_path = os.path.join(config_root, "nav2", "controllers",
                                   f"{controller}.yaml")
    robot_path      = os.path.join(config_root, "robots", f"{robot}.yaml")

    # Localization fragment is conditional on the mode:
    #   - amcl         → merge amcl.yaml
    #   - slam_toolbox → merge slam_toolbox.yaml
    #   - rtabmap_*    → no fragment (RTAB-Map is configured separately
    #                     by vf_robot_slam, not by Nav2's params system)
    if localization == "amcl":
        loc_path = os.path.join(config_root, "nav2", "localization", "amcl.yaml")
    elif localization == "slam_toolbox":
        loc_path = os.path.join(config_root, "nav2", "localization", "slam_toolbox.yaml")
    else:
        loc_path = None

    # Load the robot profile with the right controller-family overrides
    family = CONTROLLER_FAMILY[controller]
    robot_rewrites = load_robot_profile(robot_path, controller_family=family)

    # Compose. Strict mode is on — any rewrite key that doesn't exist in
    # the merged base+controller+localization params will raise here with
    # a clear "did you mean ...?" suggestion.
    composed_path = compose(
        base_path        = base_path,
        controller_path  = controller_path,
        localization_path= loc_path,
        robot_rewrites   = robot_rewrites,
        label            = f"{robot}_{controller}_{localization}",
        strict           = True,
    )

    # Print a clear log line so the user can find the file later
    print(f"[vf_bringup] Composed Nav2 params: {composed_path}")
    print(f"[vf_bringup]   robot:        {robot}")
    print(f"[vf_bringup]   controller:   {controller} (family: {family})")
    print(f"[vf_bringup]   localization: {localization}")
    print(f"[vf_bringup]   rewrites:     {len(robot_rewrites)}")

    # Sidecar reminder
    autostart = autostart_str.lower() in ("true", "1", "yes")
    if controller in SIDECAR_CONTROLLERS:
        sidecar_launch = SIDECAR_CONTROLLERS[controller]
        if autostart:
            print(f"[vf_bringup] autostart_sidecar=true — including {sidecar_launch}")
        else:
            print("[vf_bringup] " + "=" * 70)
            print(f"[vf_bringup] Controller '{controller}' requires a Python sidecar.")
            print("[vf_bringup] In a SEPARATE terminal, run:")
            print("[vf_bringup]")
            print("[vf_bringup]   conda activate dl")
            print("[vf_bringup]   source ~/cogni-nav-x0/install/setup.bash")
            print(f"[vf_bringup]   ros2 launch vf_robot_controller {sidecar_launch}")
            print("[vf_bringup]")
            print("[vf_bringup] (Or relaunch this command with autostart_sidecar:=true)")
            print("[vf_bringup] " + "=" * 70)

    # Stash the composed file path in a launch configuration so the rest
    # of the launch description can use it
    return [SetLaunchConfiguration("composed_params_file", composed_path)]


# ============================================================================
# generate_launch_description
# ============================================================================

def generate_launch_description():

    pkg_bringup = get_package_share_directory("vf_robot_bringup")
    pkg_slam    = get_package_share_directory("vf_robot_slam")

    # ── Argument declarations ────────────────────────────────────────────

    declare_robot = DeclareLaunchArgument(
        "robot",
        default_value="virofighter",
        choices=VALID_ROBOTS,
        description=f"Robot profile. One of: {VALID_ROBOTS}",
    )

    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="vf_inference",
        choices=VALID_CONTROLLERS,
        description=f"Controller plugin. One of: {VALID_CONTROLLERS}",
    )

    declare_localization = DeclareLaunchArgument(
        "localization",
        default_value="rtabmap_loc",
        choices=VALID_LOCALIZATIONS,
        description=f"SLAM/localization mode. One of: {VALID_LOCALIZATIONS}",
    )

    declare_camera = DeclareLaunchArgument(
        "camera",
        default_value="dual",
        choices=["d435i", "d455", "dual"],
        description="Camera configuration for depth_to_scan and RTAB-Map",
    )

    declare_scan_method = DeclareLaunchArgument(
        "scan_method",
        default_value="pc2scan",
        choices=["dimg", "pc2scan"],
        description="Depth-to-scan conversion method",
    )

    declare_merge_scans = DeclareLaunchArgument(
        "merge_scans",
        default_value="true",
        choices=["true", "false"],
        description="Merge dual camera scans into single /scan topic",
    )

    declare_map_name = DeclareLaunchArgument(
        "map_name",
        default_value="default_map",
        description="Map folder name (RTAB-Map slam/loc modes)",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to .yaml map file (AMCL mode only)",
    )

    declare_maps_dir = DeclareLaunchArgument(
        "maps_dir",
        default_value=os.path.join(os.path.expanduser("~"), "cogni-nav-x0", "maps"),
        description="Base directory for all maps",
    )

    declare_new_map = DeclareLaunchArgument(
        "new_map",
        default_value="true",
        choices=["true", "false"],
        description="RTAB-Map SLAM: delete existing .db and start fresh",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use Gazebo clock (true) or wall clock (false)",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz with Nav2 panels",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_bringup, "rviz", "vf_bringup.rviz"),
        description="Full path to RViz config file",
    )

    declare_autostart_sidecar = DeclareLaunchArgument(
        "autostart_sidecar",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Auto-include the Python sidecar launch from vf_robot_controller "
            "when running vf_inference/vf_collect/vf_passive. Default false: "
            "the user is reminded to start the sidecar in a separate "
            "(conda dl) terminal."
        ),
    )

    # ── Composer step (must run before any node that needs the params) ──

    compose_op = OpaqueFunction(function=_compose_action)

    # ── depth_to_scan — always runs (all 4 modes need /scan) ─────────────

    depth_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, "launch", "depth_to_scan.launch.py")
        ),
        launch_arguments={
            "method":       LaunchConfiguration("scan_method"),
            "camera":       LaunchConfiguration("camera"),
            "merge_scans":  LaunchConfiguration("merge_scans"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ── Mode 1: RTAB-Map SLAM ────────────────────────────────────────────

    rtabmap_slam_branch = GroupAction(
        condition=LaunchConfigurationEquals("localization", "rtabmap_slam"),
        actions=[
            LogInfo(msg="[vf_bringup] localization: rtabmap_slam (build new map)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, "launch", "rtabmap_slam.launch.py")
                ),
                launch_arguments={
                    "camera":       LaunchConfiguration("camera"),
                    "map_name":     LaunchConfiguration("map_name"),
                    "maps_dir":     LaunchConfiguration("maps_dir"),
                    "new_map":      LaunchConfiguration("new_map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz":         "false",  # bringup owns RViz
                }.items(),
            ),
        ],
    )

    # ── Mode 2: RTAB-Map Localization ────────────────────────────────────

    rtabmap_loc_branch = GroupAction(
        condition=LaunchConfigurationEquals("localization", "rtabmap_loc"),
        actions=[
            LogInfo(msg="[vf_bringup] localization: rtabmap_loc (load existing map)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, "launch", "rtabmap_loc.launch.py")
                ),
                launch_arguments={
                    "camera":       LaunchConfiguration("camera"),
                    "map_name":     LaunchConfiguration("map_name"),
                    "maps_dir":     LaunchConfiguration("maps_dir"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz":         "false",
                }.items(),
            ),
        ],
    )

    # ── Mode 3: AMCL + map_server ────────────────────────────────────────

    amcl_branch = GroupAction(
        condition=LaunchConfigurationEquals("localization", "amcl"),
        actions=[
            LogInfo(msg="[vf_bringup] localization: amcl (load 2D occupancy map)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "localization_launch.py")
                ),
                launch_arguments={
                    "map":          LaunchConfiguration("map"),
                    "params_file":  LaunchConfiguration("composed_params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
    )

    # ── Mode 4: SLAM Toolbox ─────────────────────────────────────────────

    slam_toolbox_branch = GroupAction(
        condition=LaunchConfigurationEquals("localization", "slam_toolbox"),
        actions=[
            LogInfo(msg="[vf_bringup] localization: slam_toolbox (laser-based SLAM)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "slam_launch.py")
                ),
                launch_arguments={
                    "params_file":  LaunchConfiguration("composed_params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
    )

    # ── Nav2 stack — always runs ─────────────────────────────────────────

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file":  LaunchConfiguration("composed_params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # ── RViz — optional ──────────────────────────────────────────────────

    rviz_branch = GroupAction(
        condition=IfCondition(LaunchConfiguration("rviz")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, "launch", "rviz_launch.py")
                ),
                launch_arguments={
                    "rviz_config":  LaunchConfiguration("rviz_config"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ],
    )

    # ── Assemble the launch description ──────────────────────────────────

    return LaunchDescription([
        # Arguments
        declare_robot,
        declare_controller,
        declare_localization,
        declare_camera,
        declare_scan_method,
        declare_merge_scans,
        declare_map_name,
        declare_map,
        declare_maps_dir,
        declare_new_map,
        declare_use_sim_time,
        declare_rviz,
        declare_rviz_config,
        declare_autostart_sidecar,

        # Compose params first — produces composed_params_file LaunchConfiguration
        compose_op,

        # depth_to_scan — all modes need /scan
        depth_to_scan,

        # Exactly ONE of these branches activates based on `localization:=`
        rtabmap_slam_branch,
        rtabmap_loc_branch,
        amcl_branch,
        slam_toolbox_branch,

        # Nav2 stack — always runs, reads composed_params_file
        navigation,

        # RViz — optional
        rviz_branch,
    ])
