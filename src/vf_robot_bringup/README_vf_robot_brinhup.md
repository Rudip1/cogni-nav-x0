# 🤖 `vf_robot_bringup`

> **ViroFighter UVC-1 Robot — Navigation Bringup Package**
> ROS 2 Humble · Nav2 · RTAB-Map · AMCL · SLAM Toolbox

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation-green?style=flat-square)](https://navigation.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green?style=flat-square)](LICENSE)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Quick Start](#-quick-start)
- [All 4 Modes](#-all-4-modes)
- [Package Structure](#-package-structure)
- [Arguments Reference](#-arguments-reference)
- [Architecture](#️-architecture)
- [Nav2 Parameters](#-nav2-parameters)
- [Using with VFRobotController](#-using-with-vfrobotcontroller)
- [RViz Controls](#-rviz-controls)
- [Verification](#-verification)
- [Troubleshooting](#-troubleshooting)
- [License](#-license)

---

## 🌟 Overview

`vf_robot_bringup` is the **single entry point** for all ViroFighter navigation. One launch file, one `mode` argument — it orchestrates SLAM/localization, depth-to-scan, Nav2, and RViz automatically.

**What it does:**

```
bringup_launch.py  mode:=<mode>
       │
       ├── depth_to_scan  (from vf_robot_slam)     → /scan
       ├── SLAM / Localization  (mode-specific)     → /map + map→odom TF
       ├── Nav2 stack  (planners, controllers, BT)  → autonomous navigation
       └── RViz  (optional)                         → visualization + goal setting
```

**What it does NOT contain:**

| Responsibility | Lives in |
|---|---|
| URDF / meshes / sensor TF | `vf_robot_description` |
| Gazebo worlds / simulation | `vf_robot_gazebo` |
| RTAB-Map / depth-to-scan nodes | `vf_robot_slam` |
| Controller plugin C++ / training | `vf_robot_controller` |
| Map files (.db, .pgm, .yaml) | `~/cogni-nav-x0/maps/` |

---

## ⚡ Quick Start

### Prerequisites

```bash
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-rtabmap-ros \
                 ros-humble-depthimage-to-laserscan
```

### Build

```bash
cd ~/cogni-nav-x0
colcon build --packages-select vf_robot_bringup --symlink-install
source install/setup.bash
```

### Launch (2 terminals)

```bash
# Terminal 1: Gazebo
ros2 launch vf_robot_gazebo vf_my_world_xacro.launch.py

# Terminal 2: Everything else
ros2 launch vf_robot_bringup bringup_launch.py mode:=rtabmap_slam camera:=dual map_name:=my_office
```

In RViz: click **Nav2 Goal** → click on the map → robot navigates while building the map.

---

## 🗺️ All 4 Modes

### Mode 1 — RTAB-Map SLAM (build a new map)

```bash
# New map
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=rtabmap_slam camera:=dual map_name:=my_office

# Continue existing map
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=rtabmap_slam camera:=dual map_name:=my_office new_map:=false
```

When done: Ctrl+C saves `my_office.db` automatically. To export 2D map for AMCL:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/cogni-nav-x0/maps/my_office/my_office
```

### Mode 2 — RTAB-Map Localization (navigate in existing map)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=rtabmap_loc camera:=dual map_name:=my_office
```

Requires `my_office.db` from a prior SLAM session.

### Mode 3 — AMCL (navigate in 2D map)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=amcl camera:=dual \
    map:=$HOME/cogni-nav-x0/maps/my_office/my_office.yaml
```

Requires `.pgm` + `.yaml` files. In RViz, use **2D Pose Estimate** to set initial robot position.

### Mode 4 — SLAM Toolbox (laser-based SLAM)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=slam_toolbox camera:=dual
```

Builds a map from `/scan` data. Good alternative when RTAB-Map visual SLAM struggles.

### Real Robot (any mode)

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=rtabmap_loc camera:=dual map_name:=my_office use_sim_time:=false
```

---

## 📁 Package Structure

```
vf_robot_bringup/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── nav2_params.yaml              # Nav2 params tuned for ViroFighter
├── rviz/
│   └── vf_bringup.rviz               # RViz with Nav2 panels
├── launch/
│   ├── bringup_launch.py             # THE entry point (all 4 modes)
│   ├── navigation_launch.py          # Nav2 stack (controller, planner, BT, etc.)
│   ├── localization_launch.py        # AMCL + map_server (Mode 3)
│   ├── slam_launch.py                # SLAM Toolbox (Mode 4)
│   └── rviz_launch.py                # Standalone RViz
├── resource/
│   └── vf_robot_bringup
└── vf_robot_bringup/
    └── __init__.py
```

---

## ⚙️ Arguments Reference

### `bringup_launch.py`

| Argument | Values | Default | Description |
|---|---|---|---|
| `mode` | `rtabmap_slam`, `rtabmap_loc`, `amcl`, `slam_toolbox` | `rtabmap_slam` | Navigation mode |
| `camera` | `d435i`, `d455`, `dual` | `dual` | Camera config for depth-to-scan |
| `scan_method` | `dimg`, `pc2scan` | `pc2scan` | Depth-to-scan method |
| `merge_scans` | `true`, `false` | `true` | Merge dual scans into /scan |
| `map_name` | string | `default_map` | Map folder name (RTAB-Map modes) |
| `map` | path | `""` | Full path to .yaml map (AMCL mode) |
| `maps_dir` | path | `~/cogni-nav-x0/maps` | Base maps directory |
| `new_map` | `true`, `false` | `true` | Fresh map or continue (RTAB-Map SLAM) |
| `params_file` | path | `config/nav2_params.yaml` | Nav2 parameters file |
| `use_sim_time` | `true`, `false` | `true` | Simulation or real robot |
| `rviz` | `true`, `false` | `true` | Launch RViz |
| `rviz` | path | `rviz/vf_bringup.rviz` | RViz config file |

---

## 🏗️ Architecture

### What `bringup_launch.py` starts per mode

```
                           ┌─────────────────────────────────────┐
                           │        bringup_launch.py            │
                           │           mode:=???                 │
                           └──────────┬──────────────────────────┘
                                      │
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
     ┌────────▼────────┐   ┌─────────▼─────────┐   ┌────────▼────────┐
     │  depth_to_scan  │   │  Mode-specific     │   │  Nav2 stack     │
     │  (ALL modes)    │   │  map provider      │   │  (ALL modes)    │
     │                 │   │                    │   │                 │
     │  from           │   │  rtabmap_slam  OR  │   │  controller     │
     │  vf_robot_slam  │   │  rtabmap_loc   OR  │   │  planner        │
     │                 │   │  amcl+map_srv  OR  │   │  behaviors      │
     │  → /scan        │   │  slam_toolbox      │   │  bt_navigator   │
     └─────────────────┘   │                    │   │  vel_smoother   │
                           │  → /map            │   │  lifecycle_mgr  │
                           │  → map→odom TF     │   └─────────────────┘
                           └────────────────────┘
```

### Dependency chain

```
vf_robot_description ──► vf_robot_gazebo ──► [Gazebo running]
         │                                         │
         └──► vf_robot_slam ◄──────────────────────┘
                    │
                    └──► vf_robot_bringup ◄── vf_robot_controller (plugin)
```

### Who publishes what

| Output | Mode 1 (SLAM) | Mode 2 (Loc) | Mode 3 (AMCL) | Mode 4 (SlamTB) |
|---|---|---|---|---|
| `/map` | RTAB-Map | RTAB-Map (.db) | map_server (.pgm) | SLAM Toolbox |
| `map→odom` TF | RTAB-Map | RTAB-Map | AMCL | SLAM Toolbox |
| `/scan` | depth_to_scan | depth_to_scan | depth_to_scan | depth_to_scan |
| `/odom` | Gazebo | Gazebo | Gazebo | Gazebo |
| `odom→base_footprint` | Gazebo | Gazebo | Gazebo | Gazebo |
| sensor TFs | robot_state_pub | robot_state_pub | robot_state_pub | robot_state_pub |

---

## 🔧 Nav2 Parameters

The default `config/nav2_params.yaml` is tuned for ViroFighter:

| Parameter | Value | Why |
|---|---|---|
| `max_vel_x` | 0.3 m/s | UVC disinfection robot — not a racing bot |
| `max_vel_theta` | 1.0 rad/s | Conservative turning for stability |
| `footprint` | `[0.1,0.2], [0.1,-0.2], [-0.5,-0.2], [-0.5,0.2]` | ViroFighter body shape |
| `inflation_radius` | 0.55 m | Slightly larger than half robot width |
| `laser_max_range` | 6.0 m | Matches RealSense depth camera range |
| `base_frame_id` | `base_footprint` | Matches Gazebo diff drive output |

### Custom params file

Pass any params file to override defaults:

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=rtabmap_loc camera:=dual map_name:=my_office \
    params_file:=/path/to/your/custom_params.yaml
```

---

## 🎮 Using with VFRobotController

To use your custom `vf_robot_controller::VFRobotController` plugin instead of DWB:

1. Edit the `FollowPath` section in your params file:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "vf_robot_controller::VFRobotController"
      controller_mode: "inference"   # or "fixed" or "collect"
      # ... your VFRobotController params ...
```

2. Launch with your params:

```bash
ros2 launch vf_robot_bringup bringup_launch.py \
    mode:=rtabmap_loc camera:=dual map_name:=my_office \
    params_file:=$HOME/cogni-nav-x0/src/vf_robot_controller/config/tb3_waffle_params.yaml
```

3. For inference mode, also start the feature extractor and inference node:

```bash
ros2 run vf_robot_controller feature_extractor.py &
ros2 run vf_robot_controller inference_node.py
```

---

## 🖱️ RViz Controls

| Button | What it does | Which modes |
|---|---|---|
| **2D Pose Estimate** | Sets initial robot pose → publishes `/initialpose` | AMCL mode (required) |
| **Nav2 Goal** | Sets navigation target → publishes `/goal_pose` | All modes |
| **Publish Point** | Click to publish a point (debugging) | All modes |

### Fixed Frame selection

| Mode | Fixed Frame |
|---|---|
| All modes (with map) | `map` (default in rviz config) |
| No map yet (early SLAM) | `odom` — switch to `map` once it appears |

---

## ✅ Verification

After launching, verify the full stack is working:

```bash
# 1. /map topic exists
ros2 topic hz /map

# 2. map→odom TF exists
ros2 run tf2_ros tf2_echo map odom

# 3. /scan is publishing
ros2 topic hz /scan

# 4. Nav2 nodes are active
ros2 lifecycle get /controller_server    # should be: active
ros2 lifecycle get /planner_server       # should be: active
ros2 lifecycle get /bt_navigator         # should be: active

# 5. Full TF chain
ros2 run tf2_tools view_frames

# 6. All critical topics
ros2 topic list | grep -E "^/(map|scan|odom|plan|local_plan|cmd_vel)$"
```

---

## 🐛 Troubleshooting

### Nav2 nodes stuck in UNCONFIGURED state

**Cause:** `map→odom` TF not available. Lifecycle manager waits for the full TF chain before activating nodes.

**Fix:** Ensure your mode-specific SLAM/localization is running and the `map` frame has appeared:

```bash
ros2 run tf2_ros tf2_echo map odom
# If this times out: the SLAM/localization node hasn't initialized yet
# For RTAB-Map: wait ~10 seconds, check /rtabmap/info topic
```

### Robot doesn't move after setting Nav2 Goal

Check the full chain:

```bash
# Is controller receiving a path?
ros2 topic hz /plan           # planner output
ros2 topic hz /local_plan     # controller output
ros2 topic hz /cmd_vel        # final velocity to robot

# Is the costmap clear?
# In RViz, enable Local Costmap display — robot must not be inside an obstacle
```

### AMCL: robot position jumps wildly

Initial pose not set. Click **2D Pose Estimate** in RViz and click the approximate robot position on the map.

### "map yaml file not found" in AMCL mode

Provide the full path to the `.yaml` file:

```bash
ros2 launch vf_robot_bringup bringup_launch.py mode:=amcl \
    map:=$HOME/cogni-nav-x0/maps/my_office/my_office.yaml
```

### Controller plugin not found

If using `vf_robot_controller::VFRobotController`, ensure it's built and sourced:

```bash
colcon build --packages-select vf_robot_controller --symlink-install
source install/setup.bash
```

### Costmap shows no obstacles

Verify `/scan` is publishing and has valid data:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once | grep -E "range_min|range_max|ranges"
```

---

## 📄 License

Apache 2.0

---

## 👤 Maintainer

**Pravin Oli** — olipravin18@gmail.com
Project: **cogni-nav-x0** | Package: `vf_robot_bringup`
